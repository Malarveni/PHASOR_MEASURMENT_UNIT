#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <math.h>
#include <time.h>
#include <esp_task_wdt.h>

// ==================== OLED DISPLAY CONFIGURATION ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_SDA 21
#define OLED_SCL 22

TwoWire I2C_OLED = TwoWire(0);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, -1);

// ==================== CONFIGURATION ====================
const char* ssid = "WIFI_NAME";
const char* password = "PASSWORD";

// IMPORTANT: Replace with your actual Google Apps Script URL after deploying
const String GOOGLE_SCRIPT_URL = "YOUR_SCRIPT_URL";

String API_KEY = "";
String SECRET_KEY = "";

// Pin Definitions
#define ZCD_PIN_R   26
#define ZCD_PIN_Y   27
#define ZCD_PIN_B   14
#define RELAY_PIN_R 25
#define RELAY_PIN_Y 33
#define RELAY_PIN_B 32
#define CURRENT_PIN_R 34
#define CURRENT_PIN_Y 35
#define CURRENT_PIN_B 36
#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define VOLTAGE_R    39
#define VOLTAGE_Y    4
#define VOLTAGE_B    2

// Sensor Constants
const float CURRENT_SENSITIVITY = 0.100;
const float ADC_REF_VOLTAGE = 3.3;
const float ADC_RESOLUTION = 4095.0;
const float SQRT3 = 1.7320508;
const float VOLTAGE_DIVIDER_RATIO = 10.0;

// Calibration
float CURRENT_OFFSET_R = 1.65;
float CURRENT_OFFSET_Y = 1.65;
float CURRENT_OFFSET_B = 1.65;

// Attack Detection Thresholds (RELAXED for stability)
const float MAX_PHASE_SUM_DEVIATION = 15.0;  // Increased from 5
const float MAX_CURRENT_CHANGE_PER_SECOND = 50.0;  // Increased from 30
const float MAX_GPS_JUMP_KM = 50.0;  // Increased from 10
const float KCL_THRESHOLD = 5.0;  // Increased from 1.5

// Data Logging - REDUCED to save memory
#define MAX_LOG_ENTRIES 50  // Was 200

struct LogEntry {
  unsigned long timestamp;
  float current_R, current_Y, current_B;
  float voltage_R, voltage_Y, voltage_B;
  float phase_RY, phase_YB, phase_BR;
  float frequency;
  float lat, lng;
  char time_str[20];
  char date_str[20];
  bool attack_detected;
  char attack_type[50];
};

LogEntry logData[MAX_LOG_ENTRIES];
int logIndex = 0;

// Global Variables
float current_R = 0.0, current_Y = 0.0, current_B = 0.0;
float voltage_R = 0.0, voltage_Y = 0.0, voltage_B = 0.0;
float phase_diff_RY = 0.0, phase_diff_YB = 0.0, phase_diff_BR = 0.0;
float frequency = 50.0;
float gps_lat = 0.0, gps_lng = 0.0;
String gps_time = "00:00:00";
String gps_date = "00/00/00";

bool attack_detected = false;
String attack_reason = "";
int attack_count = 0;
bool relay_R = false, relay_Y = false, relay_B = false;

// GPS Object
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
WebServer server(80);

// Phase Angle Variables with improved stability
volatile unsigned long time_R = 0, time_Y = 0, time_B = 0;
volatile bool new_pulse_R = false, new_pulse_Y = false, new_pulse_B = false;
unsigned long last_freq_time = 0;
const unsigned long DEBOUNCE_US = 1000;  // Increased for stability
unsigned long last_phase_calc = 0;
unsigned long last_good_freq_time = 0;
float stable_frequency = 50.0;
int freq_samples = 0;

// Display control
unsigned long last_page_switch = 0;
const unsigned long PAGE_DISPLAY_TIME = 4000;

// Previous values for attack detection
float prev_current_R = 0, prev_current_Y = 0, prev_current_B = 0;
unsigned long last_current_time = 0;
float prev_lat = 0, prev_lng = 0;
int attack_count_ramp = 0;

// System health
unsigned long last_health_check = 0;
unsigned long last_wifi_check = 0;
int reboot_count = 0;

// ==================== HELPER FUNCTIONS ====================

float calculateDistanceBetweenGPS(float lat1, float lng1, float lat2, float lng2) {
  if (lat1 == 0 || lng1 == 0 || lat2 == 0 || lng2 == 0) return 0;
  
  float R = 6371;
  float dlat = (lat2 - lat1) * PI / 180.0;
  float dlng = (lng2 - lng1) * PI / 180.0;
  float a = sin(dlat/2) * sin(dlat/2) +
            cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
            sin(dlng/2) * sin(dlng/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// ==================== FIXED ATTACK DETECTION ====================
void checkForAttacks() {
  attack_detected = false;
  attack_reason = "";
  
  // 1. FIXED: PHASE ANGLE SUM CHECK with validation
  static int valid_phase_readings = 0;
  if (frequency > 47.0 && frequency < 53.0 && stable_frequency > 0) {
    float phase_sum = phase_diff_RY + phase_diff_YB + phase_diff_BR;
    if (phase_sum > 0 && phase_sum < 720) {  // Valid range check
      if (abs(phase_sum - 360.0) > MAX_PHASE_SUM_DEVIATION && valid_phase_readings > 10) {
        attack_detected = true;
        attack_reason = "PHASE_ANGLE_ATTACK: Sum=" + String(phase_sum, 1) + "°";
        Serial.println("⚠️ CYBER ATTACK DETECTED: " + attack_reason);
      }
      valid_phase_readings++;
      if (valid_phase_readings > 100) valid_phase_readings = 50;
    }
  } else {
    valid_phase_readings = 0;
  }
  
  // 2. FIXED: Proper 3-phase current balance check (Neutral current)
  if (current_R > 0.1 || current_Y > 0.1 || current_B > 0.1) {
    // Calculate neutral current for balanced 3-phase system
    float neutral_current = sqrt(
      current_R*current_R + current_Y*current_Y + current_B*current_B - 
      current_R*current_Y - current_Y*current_B - current_B*current_R
    ) / SQRT3;
    
    if (neutral_current > KCL_THRESHOLD && neutral_current > 1.0) {
      attack_detected = true;
      attack_reason = "NEUTRAL_CURRENT_ATTACK: In=" + String(neutral_current, 2) + "A";
      Serial.println("⚠️ CYBER ATTACK DETECTED: " + attack_reason);
    }
  }
  
  // 3. FIXED: Rate of change with debouncing
  unsigned long now = millis();
  if (last_current_time > 0) {
    float dt = (now - last_current_time) / 1000.0;
    if (dt > 0.1 && dt < 5.0) {
      float dI_R = abs(current_R - prev_current_R) / dt;
      float dI_Y = abs(current_Y - prev_current_Y) / dt;
      float dI_B = abs(current_B - prev_current_B) / dt;
      
      if (dI_R > MAX_CURRENT_CHANGE_PER_SECOND && dI_R < 1000 && current_R > 2.0) {
        attack_count_ramp++;
        if (attack_count_ramp > 2) {
          attack_detected = true;
          attack_reason = "RAPID_CURRENT_SPIKE_R: " + String(dI_R, 1) + "A/s";
          Serial.println("⚠️ CYBER ATTACK DETECTED: " + attack_reason);
        }
      } else if (dI_Y > MAX_CURRENT_CHANGE_PER_SECOND && dI_Y < 1000 && current_Y > 2.0) {
        attack_count_ramp++;
        if (attack_count_ramp > 2) {
          attack_detected = true;
          attack_reason = "RAPID_CURRENT_SPIKE_Y: " + String(dI_Y, 1) + "A/s";
          Serial.println("⚠️ CYBER ATTACK DETECTED: " + attack_reason);
        }
      } else if (dI_B > MAX_CURRENT_CHANGE_PER_SECOND && dI_B < 1000 && current_B > 2.0) {
        attack_count_ramp++;
        if (attack_count_ramp > 2) {
          attack_detected = true;
          attack_reason = "RAPID_CURRENT_SPIKE_B: " + String(dI_B, 1) + "A/s";
          Serial.println("⚠️ CYBER ATTACK DETECTED: " + attack_reason);
        }
      } else {
        if (attack_count_ramp > 0) attack_count_ramp--;
      }
    }
  }
  
  // 4. FIXED: Frequency anomaly with averaging
  if (stable_frequency > 0) {
    if (stable_frequency < 45.0 || stable_frequency > 55.0) {
      attack_detected = true;
      attack_reason = "FREQUENCY_ATTACK: Frequency = " + String(stable_frequency, 2) + "Hz";
      Serial.println("⚠️ CYBER ATTACK DETECTED: " + attack_reason);
    }
  }
  
  // Update previous values
  prev_current_R = current_R;
  prev_current_Y = current_Y;
  prev_current_B = current_B;
  prev_lat = gps_lat;
  prev_lng = gps_lng;
  last_current_time = now;
  
  if (attack_detected) {
    attack_count++;
    logCurrentData();
  }
}

// ==================== API KEY SETUP ====================
void setupAPIKeys() {
  Serial.println("\n==========================================");
  Serial.println("🔐 API KEY SETUP");
  Serial.println("==========================================");
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Enter API Key");
  display.println("in Serial Monitor");
  display.display();
  
  Serial.print("Enter your API_KEY (or press Enter for default): ");
  
  unsigned long startTime = millis();
  while (!Serial.available() && (millis() - startTime) < 30000) {
    delay(100);
  }
  
  if (Serial.available()) {
    API_KEY = Serial.readStringUntil('\n');
    API_KEY.trim();
  }
  
  if (API_KEY.length() < 4) {
    Serial.println("⚠️ Using default API_KEY");
    API_KEY = "PMU_DEFAULT_KEY";
  }
  
  Serial.print("Enter your SECRET_KEY (or press Enter for default): ");
  startTime = millis();
  while (!Serial.available() && (millis() - startTime) < 30000) {
    delay(100);
  }
  
  if (Serial.available()) {
    SECRET_KEY = Serial.readStringUntil('\n');
    SECRET_KEY.trim();
  }
  
  if (SECRET_KEY.length() < 4) {
    SECRET_KEY = "PMU_SECRET";
  }
  
  Serial.println("\n✅ KEYS CONFIGURED!");
  Serial.print("API_KEY: "); Serial.println(API_KEY);
  Serial.println("==========================================\n");
  delay(1000);
}

// ==================== RELAY CONTROL ====================
void setRelay(int relay_pin, bool &relay_state, bool turn_on, String phase_name) {
  relay_state = turn_on;
  digitalWrite(relay_pin, turn_on ? LOW : HIGH);
  Serial.println("✅ Relay " + phase_name + (turn_on ? " ON" : " OFF"));
}

// ==================== SERIAL COMMANDS ====================
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command.length() == 0) return;
    
    Serial.println("\n📟 Command: " + command);
    
    if (command == "HELP" || command == "?") {
      Serial.println("\n========== COMMANDS ==========");
      Serial.println("RELAY R ON/OFF/TOGGLE");
      Serial.println("RELAY Y ON/OFF/TOGGLE");
      Serial.println("RELAY B ON/OFF/TOGGLE");
      Serial.println("RELAY ALL ON/OFF");
      Serial.println("STATUS - Show system status");
      Serial.println("ATTACK - Show attack statistics");
      Serial.println("CLEAR - Clear screen");
      Serial.println("REBOOT - Restart system");
      Serial.println("HELP - This menu");
      Serial.println("==============================\n");
      return;
    }
    
    if (command == "CLEAR") {
      for (int i = 0; i < 30; i++) Serial.println();
      return;
    }
    
    if (command == "REBOOT") {
      Serial.println("🔄 Rebooting...");
      delay(100);
      ESP.restart();
      return;
    }
    
    if (command == "STATUS") {
      Serial.println("\n========== SYSTEM STATUS ==========");
      Serial.printf("Relay R: %s\n", relay_R ? "ON" : "OFF");
      Serial.printf("Relay Y: %s\n", relay_Y ? "ON" : "OFF");
      Serial.printf("Relay B: %s\n", relay_B ? "ON" : "OFF");
      Serial.printf("Attack Detected: %s\n", attack_detected ? "YES" : "NO");
      Serial.printf("Attack Count: %d\n", attack_count);
      Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
      Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
      if (attack_detected) Serial.printf("Last Attack: %s\n", attack_reason.c_str());
      Serial.println("===================================\n");
      return;
    }
    
    if (command == "ATTACK") {
      Serial.println("\n========== ATTACK STATISTICS ==========");
      Serial.printf("Total Attacks Detected: %d\n", attack_count);
      Serial.printf("Current Attack Status: %s\n", attack_detected ? "ACTIVE" : "NONE");
      if (attack_reason.length() > 0) Serial.printf("Last Attack: %s\n", attack_reason.c_str());
      Serial.println("=======================================\n");
      return;
    }
    
    // Relay commands
    if (command == "RELAY ALL ON") {
      setRelay(RELAY_PIN_R, relay_R, true, "R");
      setRelay(RELAY_PIN_Y, relay_Y, true, "Y");
      setRelay(RELAY_PIN_B, relay_B, true, "B");
      return;
    }
    if (command == "RELAY ALL OFF") {
      setRelay(RELAY_PIN_R, relay_R, false, "R");
      setRelay(RELAY_PIN_Y, relay_Y, false, "Y");
      setRelay(RELAY_PIN_B, relay_B, false, "B");
      return;
    }
    
    if (command.startsWith("RELAY")) {
      if (command.indexOf("R") >= 0) {
        if (command.indexOf("ON") >= 0) setRelay(RELAY_PIN_R, relay_R, true, "R");
        else if (command.indexOf("OFF") >= 0) setRelay(RELAY_PIN_R, relay_R, false, "R");
        else if (command.indexOf("TOGGLE") >= 0) setRelay(RELAY_PIN_R, relay_R, !relay_R, "R");
      }
      else if (command.indexOf("Y") >= 0) {
        if (command.indexOf("ON") >= 0) setRelay(RELAY_PIN_Y, relay_Y, true, "Y");
        else if (command.indexOf("OFF") >= 0) setRelay(RELAY_PIN_Y, relay_Y, false, "Y");
        else if (command.indexOf("TOGGLE") >= 0) setRelay(RELAY_PIN_Y, relay_Y, !relay_Y, "Y");
      }
      else if (command.indexOf("B") >= 0) {
        if (command.indexOf("ON") >= 0) setRelay(RELAY_PIN_B, relay_B, true, "B");
        else if (command.indexOf("OFF") >= 0) setRelay(RELAY_PIN_B, relay_B, false, "B");
        else if (command.indexOf("TOGGLE") >= 0) setRelay(RELAY_PIN_B, relay_B, !relay_B, "B");
      }
    }
  }
}

// ==================== CALIBRATION ====================
void calibrateCurrentSensors() {
  Serial.println("Calibrating current sensors...");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.println("Ensure NO load");
  display.display();
  
  float sum_R = 0, sum_Y = 0, sum_B = 0;
  int samples = 100;  // Reduced for faster calibration
  
  for (int i = 0; i < samples; i++) {
    sum_R += (analogRead(CURRENT_PIN_R) / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    sum_Y += (analogRead(CURRENT_PIN_Y) / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    sum_B += (analogRead(CURRENT_PIN_B) / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    delay(3);
    yield();  // Allow other tasks
  }
  
  CURRENT_OFFSET_R = sum_R / samples;
  CURRENT_OFFSET_Y = sum_Y / samples;
  CURRENT_OFFSET_B = sum_B / samples;
  
  Serial.print("Offsets - R:");
  Serial.print(CURRENT_OFFSET_R, 3);
  Serial.print("V Y:");
  Serial.print(CURRENT_OFFSET_Y, 3);
  Serial.print("V B:");
  Serial.println(CURRENT_OFFSET_B, 3);
  delay(1000);
}

// ==================== MEASUREMENT FUNCTIONS ====================
float readVoltageRMS(int pin) {
  float sum_squares = 0;
  int samples = 100;  // Reduced for performance
  
  for (int i = 0; i < samples; i++) {
    int adc_value = analogRead(pin);
    float voltage = (adc_value / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    voltage = voltage * VOLTAGE_DIVIDER_RATIO;
    sum_squares += (voltage * voltage);
    delayMicroseconds(100);
  }
  return sqrt(sum_squares / samples);
}

float readCurrentRMS(int pin, float offset_voltage) {
  float sum_squares = 0;
  int valid_samples = 0;
  
  for (int i = 0; i < 50; i++) {  // Reduced for performance
    int adc_value = analogRead(pin);
    float voltage = (adc_value / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    float current = (voltage - offset_voltage) / CURRENT_SENSITIVITY;
    if (fabs(current) < 50) {  // Sanity check
      sum_squares += (current * current);
      valid_samples++;
    }
    delayMicroseconds(100);
  }
  
  if (valid_samples == 0) return 0;
  float rms_current = sqrt(sum_squares / valid_samples);
  return (rms_current < 0.05) ? 0.0 : rms_current;
}

// ==================== IMPROVED INTERRUPTS ====================
void IRAM_ATTR isr_R() {
  unsigned long now = micros();
  unsigned long diff = now - time_R;
  if (diff > DEBOUNCE_US && diff < 100000) {
    time_R = now;
    new_pulse_R = true;
    
    if (last_freq_time != 0) {
      float period = (now - last_freq_time) / 1000000.0;
      if (period > 0.016 && period < 0.025) {  // 40-60Hz range
        float inst_freq = 1.0 / period;
        // Moving average for stable frequency
        if (freq_samples < 10) {
          stable_frequency = (stable_frequency * freq_samples + inst_freq) / (freq_samples + 1);
          freq_samples++;
        } else {
          stable_frequency = stable_frequency * 0.9 + inst_freq * 0.1;
        }
        frequency = stable_frequency;
      }
    }
    last_freq_time = now;
  }
}

void IRAM_ATTR isr_Y() {
  unsigned long now = micros();
  unsigned long diff = now - time_Y;
  if (diff > DEBOUNCE_US && diff < 100000) {
    time_Y = now;
    new_pulse_Y = true;
  }
}

void IRAM_ATTR isr_B() {
  unsigned long now = micros();
  unsigned long diff = now - time_B;
  if (diff > DEBOUNCE_US && diff < 100000) {
    time_B = now;
    new_pulse_B = true;
  }
}

float calculatePhaseDiff(unsigned long t1, unsigned long t2, float freq) {
  if (freq <= 0) return 0;
  long time_diff = (long)(t2 - t1);
  if (time_diff < 0) time_diff += 4294967295UL;
  float period_us = (1.0 / freq) * 1000000.0;
  float phase = fmod((time_diff / period_us) * 360.0, 360.0);
  if (phase < 0) phase += 360.0;
  return phase;
}

// ==================== GPS ====================
void readGPS() {
  int available = SerialGPS.available();
  if (available > 0) {
    for (int i = 0; i < available && i < 100; i++) {
      if (gps.encode(SerialGPS.read())) {
        if (gps.location.isValid()) {
          gps_lat = gps.location.lat();
          gps_lng = gps.location.lng();
        }
        if (gps.time.isValid()) {
          char timeStr[10];
          sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
          gps_time = String(timeStr);
        }
        if (gps.date.isValid()) {
          char dateStr[11];
          sprintf(dateStr, "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year() % 100);
          gps_date = String(dateStr);
        }
      }
    }
  }
}

// ==================== OLED DISPLAY ====================
void displayPage1_Currents() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("=== CURRENTS (A) ===");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(5, 20);
  display.print("R:");
  display.print(current_R, 1);
  
  display.setCursor(5, 38);
  display.print("Y:");
  display.print(current_Y, 1);
  
  display.setCursor(5, 56);
  display.print("B:");
  display.print(current_B, 1);
  
  if (attack_detected) {
    display.setTextSize(1);
    display.fillRect(90, 50, 38, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(92, 52);
    display.print("ATTACK!");
    display.setTextColor(SSD1306_WHITE);
  }
  
  display.display();
}

void displayPage2_PhaseAngles() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("=== PHASE ANGLES ===");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(5, 20);
  display.print("R-Y: ");
  display.print(phase_diff_RY, 1);
  display.print(" deg");
  
  display.setCursor(5, 32);
  display.print("Y-B: ");
  display.print(phase_diff_YB, 1);
  display.print(" deg");
  
  display.setCursor(5, 44);
  display.print("B-R: ");
  display.print(phase_diff_BR, 1);
  display.print(" deg");
  
  float sum = phase_diff_RY + phase_diff_YB + phase_diff_BR;
  display.setCursor(5, 56);
  display.print("Sum: ");
  display.print(sum, 1);
  display.print(" deg");
  
  if (attack_detected) {
    display.fillRect(90, 50, 38, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(92, 52);
    display.print("ATTACK!");
    display.setTextColor(SSD1306_WHITE);
  }
  
  display.display();
}

void displayPage3_Frequency() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("=== FREQUENCY ===");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setTextSize(3);
  display.setCursor(20, 25);
  display.print(frequency, 1);
  display.print("Hz");
  
  display.setTextSize(1);
  display.setCursor(5, 55);
  if (frequency >= 49.5 && frequency <= 50.5) {
    display.print("Status: NORMAL");
  } else if (frequency >= 47.0 && frequency <= 53.0) {
    display.fillRect(0, 50, 128, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(5, 52);
    display.print("STATUS: WARNING");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.fillRect(0, 50, 128, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(5, 52);
    display.print("STATUS: ATTACK!");
    display.setTextColor(SSD1306_WHITE);
  }
  
  display.display();
}

void displayPage4_Voltages() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("=== VOLTAGES (V) ===");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setTextSize(2);
  display.setCursor(5, 20);
  display.print("R:");
  display.print(voltage_R, 0);
  
  display.setCursor(5, 38);
  display.print("Y:");
  display.print(voltage_Y, 0);
  
  display.setCursor(5, 56);
  display.print("B:");
  display.print(voltage_B, 0);
  
  display.display();
}

void updateOLEDDisplay() {
  static unsigned long lastDisplayUpdate = 0;
  static int page = 0;
  
  if (millis() - lastDisplayUpdate >= PAGE_DISPLAY_TIME) {
    page = (page + 1) % 4;
    
    switch(page) {
      case 0: displayPage1_Currents(); break;
      case 1: displayPage2_PhaseAngles(); break;
      case 2: displayPage3_Frequency(); break;
      case 3: displayPage4_Voltages(); break;
    }
    
    lastDisplayUpdate = millis();
  }
}

// ==================== DATA LOGGING ====================
void logCurrentData() {
  if (logIndex < MAX_LOG_ENTRIES) {
    logData[logIndex].timestamp = millis();
    logData[logIndex].current_R = current_R;
    logData[logIndex].current_Y = current_Y;
    logData[logIndex].current_B = current_B;
    logData[logIndex].voltage_R = voltage_R;
    logData[logIndex].voltage_Y = voltage_Y;
    logData[logIndex].voltage_B = voltage_B;
    logData[logIndex].phase_RY = phase_diff_RY;
    logData[logIndex].phase_YB = phase_diff_YB;
    logData[logIndex].phase_BR = phase_diff_BR;
    logData[logIndex].frequency = frequency;
    logData[logIndex].lat = gps_lat;
    logData[logIndex].lng = gps_lng;
    logData[logIndex].attack_detected = attack_detected;
    strncpy(logData[logIndex].attack_type, attack_reason.c_str(), 49);
    logData[logIndex].attack_type[49] = '\0';
    strncpy(logData[logIndex].time_str, gps_time.c_str(), 19);
    strncpy(logData[logIndex].date_str, gps_date.c_str(), 19);
    logIndex++;
    
    if (attack_detected) {
      Serial.println("🚨 ATTACK LOGGED: " + attack_reason);
    }
  } else {
    // Shift log entries if full
    for (int i = 0; i < MAX_LOG_ENTRIES - 1; i++) {
      logData[i] = logData[i + 1];
    }
    logIndex = MAX_LOG_ENTRIES - 1;
  }
}

// ==================== FIXED GOOGLE SHEETS ====================
void sendToGoogleSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ No WiFi connection, skipping upload");
    return;
  }
  
  HTTPClient http;
  http.setTimeout(5000);
  
  // Use GET method for better compatibility
  String url = GOOGLE_SCRIPT_URL + "?";
  url += "api_key=" + API_KEY;
  url += "&v_r=" + String(voltage_R, 1);
  url += "&v_y=" + String(voltage_Y, 1);
  url += "&v_b=" + String(voltage_B, 1);
  url += "&i_r=" + String(current_R, 2);
  url += "&i_y=" + String(current_Y, 2);
  url += "&i_b=" + String(current_B, 2);
  url += "&angle_ry=" + String(phase_diff_RY, 1);
  url += "&angle_yb=" + String(phase_diff_YB, 1);
  url += "&angle_br=" + String(phase_diff_BR, 1);
  url += "&freq=" + String(frequency, 1);
  url += "&lat=" + String(gps_lat, 6);
  url += "&lng=" + String(gps_lng, 6);
  url += "&time=" + gps_time;
  url += "&date=" + gps_date;
  url += "&attack=" + String(attack_detected ? 1 : 0);
  url += "&attack_reason=" + attack_reason;
  
  http.begin(url);
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    Serial.println("✅ Data sent to Google Sheets");
  } else if (httpResponseCode == 401) {
    Serial.println("❌ Authentication failed - Check your Google Apps Script deployment");
  } else {
    Serial.printf("❌ Upload error: %d\n", httpResponseCode);
  }
  
  http.end();
}

// ==================== WEB SERVER ====================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<title>3-Phase Secure PMU</title>";
  
  // Google Maps API (free, no key needed for basic)
  html += "<script src='https://maps.googleapis.com/maps/api/js?key=&callback=initMap' async defer></script>";
  
  html += "<style>";
  html += "*{margin:0;padding:0;box-sizing:border-box}";
  html += "body{font-family:'Segoe UI',Arial;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;padding:20px}";
  html += ".container{max-width:1200px;margin:auto}";
  html += ".header{text-align:center;margin-bottom:30px}";
  html += ".header h1{color:white;font-size:2em;margin-bottom:10px}";
  html += ".card{background:rgba(255,255,255,0.95);border-radius:15px;padding:20px;margin-bottom:20px;box-shadow:0 10px 30px rgba(0,0,0,0.2)}";
  html += ".attack-card{background:#ff4757;color:white;animation:pulse 1s infinite}";
  html += "@keyframes pulse{0%{opacity:1}50%{opacity:0.7}100%{opacity:1}}";
  html += ".card h2{margin-bottom:15px;color:#333;border-left:4px solid #667eea;padding-left:12px}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:15px;margin-bottom:20px}";
  html += ".metric{background:#f8f9fa;padding:15px;border-radius:10px;text-align:center}";
  html += ".metric-label{font-size:0.9em;color:#666;margin-bottom:5px}";
  html += ".metric-value{font-size:2em;font-weight:bold;color:#333}";
  html += "#map{height:400px;width:100%;border-radius:10px;margin-top:15px;margin-bottom:15px}";
  html += ".map-link{display:inline-block;padding:10px 20px;background:#4285F4;color:white;text-decoration:none;border-radius:5px;margin-top:10px}";
  html += ".map-link:hover{background:#3367D6}";
  html += "table{width:100%;border-collapse:collapse}";
  html += "th,td{padding:12px;text-align:left;border-bottom:1px solid #ddd}";
  html += "th{background:#667eea;color:white}";
  html += ".btn{padding:12px 24px;margin:5px;border:none;border-radius:8px;cursor:pointer;font-weight:bold;transition:transform 0.2s}";
  html += ".btn:hover{transform:scale(1.05)}";
  html += ".btn-on{background:#00d25b;color:white}";
  html += ".btn-off{background:#ff4757;color:white}";
  html += ".status-normal{color:#00d25b;font-weight:bold}";
  html += ".status-attack{color:#ff4757;font-weight:bold}";
  html += "</style>";
  
  // JavaScript for Google Maps
  html += "<script>";
  html += "var map;";
  html += "var marker;";
  html += "var currentLat = " + String(gps_lat, 6) + ";";
  html += "var currentLng = " + String(gps_lng, 6) + ";";
  html += "";
  html += "function initMap() {";
  html += "  var location = {lat: currentLat, lng: currentLng};";
  html += "  map = new google.maps.Map(document.getElementById('map'), {";
  html += "    zoom: 15,";
  html += "    center: location,";
  html += "    mapTypeId: google.maps.MapTypeId.ROADMAP";
  html += "  });";
  html += "  marker = new google.maps.Marker({";
  html += "    position: location,";
  html += "    map: map,";
  html += "    title: 'ESP32 PMU Location',";
  html += "    animation: google.maps.Animation.DROP";
  html += "  });";
  html += "}";
  html += "";
  html += "function updateMap() {";
  html += "  if (map && marker && currentLat != 0 && currentLng != 0) {";
  html += "    var newLocation = {lat: currentLat, lng: currentLng};";
  html += "    map.setCenter(newLocation);";
  html += "    marker.setPosition(newLocation);";
  html += "  }";
  html += "}";
  html += "";
  html += "function openGoogleMaps() {";
  html += "  if (currentLat != 0 && currentLng != 0) {";
  html += "    var url = 'https://www.google.com/maps?q=' + currentLat + ',' + currentLng;";
  html += "    window.open(url, '_blank');";
  html += "  } else {";
  html += "    alert('GPS location not available yet');";
  html += "  }";
  html += "}";
  html += "";
  html += "// Update location every 10 seconds";
  html += "setInterval(function() {";
  html += "  fetch('/location').then(r=>r.json()).then(data => {";
  html += "    if(data.lat && data.lng && data.lat != 0 && data.lng != 0) {";
  html += "      currentLat = data.lat;";
  html += "      currentLng = data.lng;";
  html += "      updateMap();";
  html += "      document.getElementById('lat-display').innerText = currentLat.toFixed(6);";
  html += "      document.getElementById('lng-display').innerText = currentLng.toFixed(6);";
  html += "    }";
  html += "  }).catch(err => console.log('Location fetch error:', err));";
  html += "}, 10000);";
  html += "</script>";
  
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<div class='header'><h1>🔒 3-Phase Secure PMU</h1><p>Real-time Power Monitoring with GPS Tracking</p></div>";
  
  if (attack_detected) {
    html += "<div class='card attack-card'><h2>🚨 CYBER ATTACK DETECTED!</h2>";
    html += "<p><strong>Attack Type:</strong> " + attack_reason + "</p>";
    html += "<p><strong>Total Attacks:</strong> " + String(attack_count) + "</p></div>";
  }
  
  // GPS Map Card
  html += "<div class='card'><h2>📍 GPS Location Tracker</h2>";
  html += "<div class='grid'>";
  html += "<div class='metric'><div class='metric-label'>Latitude</div>";
  html += "<div class='metric-value' id='lat-display'>" + String(gps_lat, 6) + "</div></div>";
  html += "<div class='metric'><div class='metric-label'>Longitude</div>";
  html += "<div class='metric-value' id='lng-display'>" + String(gps_lng, 6) + "</div></div>";
  html += "</div>";
  html += "<div id='map'></div>";
  html += "<center><button onclick='openGoogleMaps()' class='map-link'>🗺️ Open in Google Maps App</button></center>";
  html += "</div>";
  
  html += "<div class='card'><h2>📊 Live Measurements</h2>";
  html += "<div class='grid'>";
  html += "<div class='metric'><div class='metric-label'>System Status</div>";
  html += "<div class='metric-value " + String(attack_detected ? "status-attack" : "status-normal") + "'>" + (attack_detected ? "⚠️ ATTACK" : "✅ NORMAL") + "</div></div>";
  html += "<div class='metric'><div class='metric-label'>Frequency</div>";
  html += "<div class='metric-value'>" + String(frequency, 1) + "<span class='unit'>Hz</span></div></div>";
  html += "</div>";
  
  html += "<table>";
  html += "<tr><th>Parameter</th><th>Phase R</th><th>Phase Y</th><th>Phase B</th></tr>";
  html += "<tr><td>⚡ Current (A)</td><td><strong>" + String(current_R, 2) + "</strong></td><td><strong>" + String(current_Y, 2) + "</strong></td><td><strong>" + String(current_B, 2) + "</strong></td></tr>";
  html += "<tr><td>🔌 Voltage (V)</td><td><strong>" + String(voltage_R, 0) + "</strong></td><td><strong>" + String(voltage_Y, 0) + "</strong></td><td><strong>" + String(voltage_B, 0) + "</strong></td></tr>";
  html += "<tr><td>📐 Phase Angle (°)</td><td>" + String(phase_diff_RY, 1) + " (R-Y)</td><td>" + String(phase_diff_YB, 1) + " (Y-B)</td><td>" + String(phase_diff_BR, 1) + " (B-R)</td></tr>";
  html += "</table></div>";
  
  html += "<div class='card'><h2>🎛️ Relay Control</h2>";
  html += "<button class='btn btn-on' onclick='controlRelay(\"R\",1)'>🔴 R ON</button>";
  html += "<button class='btn btn-off' onclick='controlRelay(\"R\",0)'>⚫ R OFF</button>";
  html += "<button class='btn btn-on' onclick='controlRelay(\"Y\",1)'>🟡 Y ON</button>";
  html += "<button class='btn btn-off' onclick='controlRelay(\"Y\",0)'>⚫ Y OFF</button>";
  html += "<button class='btn btn-on' onclick='controlRelay(\"B\",1)'>🔵 B ON</button>";
  html += "<button class='btn btn-off' onclick='controlRelay(\"B\",0)'>⚫ B OFF</button>";
  html += "<br><br><button class='btn btn-on' onclick='controlRelay(\"ALL\",1)'>🟢 ALL ON</button>";
  html += "<button class='btn btn-off' onclick='controlRelay(\"ALL\",0)'>🔴 ALL OFF</button>";
  html += "</div>";
  
  html += "<div class='card'><h2>ℹ️ System Info</h2>";
  html += "<p>📍 Time: " + gps_time + " | Date: " + gps_date + "</p>";
  html += "<p>📡 IP: " + WiFi.localIP().toString() + " | RSSI: " + String(WiFi.RSSI()) + " dBm</p>";
  html += "</div>";
  
  html += "<script>";
  html += "function controlRelay(phase,state){";
  html += "fetch('/relay?phase='+phase+'&state='+state).then(r=>r.json()).then(d=>{";
  html += "if(d.status=='ok') location.reload();";
  html += "});}</script>";
  html += "</div></body></html>";
  
  server.send(200, "text/html", html);
}

// Add location endpoint
void handleLocation() {
  String json = "{\"lat\":" + String(gps_lat, 6) + ",\"lng\":" + String(gps_lng, 6) + "}";
  server.send(200, "application/json", json);
}

void handleRelay() {
  if (server.hasArg("phase") && server.hasArg("state")) {
    String phase = server.arg("phase");
    int state = server.arg("state").toInt();
    
    if (phase == "ALL") {
      setRelay(RELAY_PIN_R, relay_R, (state == 1), "R");
      setRelay(RELAY_PIN_Y, relay_Y, (state == 1), "Y");
      setRelay(RELAY_PIN_B, relay_B, (state == 1), "B");
    } else if (phase == "R") {
      setRelay(RELAY_PIN_R, relay_R, (state == 1), "R");
    } else if (phase == "Y") {
      setRelay(RELAY_PIN_Y, relay_Y, (state == 1), "Y");
    } else if (phase == "B") {
      setRelay(RELAY_PIN_B, relay_B, (state == 1), "B");
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\"}");
  }
}

// ==================== SYSTEM HEALTH CHECK ====================
void checkSystemHealth() {
  if (millis() - last_health_check > 60000) {
    Serial.printf("📊 System Health - Heap: %d bytes, Free PSRAM: %d bytes\n", 
                  ESP.getFreeHeap(), ESP.getFreePsram());
    
    if (ESP.getFreeHeap() < 20000) {
      Serial.println("⚠️ Low memory warning!");
    }
    
    last_health_check = millis();
  }
  
  // WiFi health check
  if (millis() - last_wifi_check > 30000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi disconnected! Reconnecting...");
      WiFi.reconnect();
    }
    last_wifi_check = millis();
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(512);
  delay(1000);
  
  Serial.println("\n🔐 Starting Secure 3-Phase PMU with Cyber Attack Detection");
  Serial.println("\n💡 Type 'HELP' for available commands!\n");
  
  // Initialize watchdog timer (safe version for all ESP32 cores)
  // Comment out if you experience issues
  /*
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,
      .idle_core_mask = (1 << CONFIG_ESP_TASK_WDT_CHECK_IDLE_CORE_0) | 
                        (1 << CONFIG_ESP_TASK_WDT_CHECK_IDLE_CORE_1),
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  */
  
  // Initialize I2C for OLED
  I2C_OLED.begin(OLED_SDA, OLED_SCL, 100000);
  server.on("/location", handleLocation);
  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("❌ OLED allocation failed!");
  } else {
    Serial.println("✅ OLED initialized successfully!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("3-Phase PMU");
    display.println("Initializing...");
    display.display();
    delay(1000);
  }
  
  // Setup API Keys
  setupAPIKeys();
  
  randomSeed(analogRead(0));
  
  // Configure Pins
  pinMode(ZCD_PIN_R, INPUT_PULLUP);
  pinMode(ZCD_PIN_Y, INPUT_PULLUP);
  pinMode(ZCD_PIN_B, INPUT_PULLUP);
  
  pinMode(RELAY_PIN_R, OUTPUT);
  pinMode(RELAY_PIN_Y, OUTPUT);
  pinMode(RELAY_PIN_B, OUTPUT);
  
  digitalWrite(RELAY_PIN_R, HIGH);
  digitalWrite(RELAY_PIN_Y, HIGH);
  digitalWrite(RELAY_PIN_B, HIGH);
  Serial.println("✅ Relays initialized to OFF state");
  
  pinMode(CURRENT_PIN_R, INPUT);
  pinMode(CURRENT_PIN_Y, INPUT);
  pinMode(CURRENT_PIN_B, INPUT);
  pinMode(VOLTAGE_R, INPUT);
  pinMode(VOLTAGE_Y, INPUT);
  pinMode(VOLTAGE_B, INPUT);
  
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Calibrate
  calibrateCurrentSensors();
  
  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ZCD_PIN_R), isr_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ZCD_PIN_Y), isr_Y, RISING);
  attachInterrupt(digitalPinToInterrupt(ZCD_PIN_B), isr_B, RISING);
  
  // WiFi
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();
  
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  Serial.print("📡 Connecting to WiFi");
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 40) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
    // esp_task_wdt_reset(); // Comment this out if watchdog is disabled
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("🌐 IP: ");
    Serial.println(WiFi.localIP());
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Connected!");
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.display();
  } else {
    Serial.println("\n⚠️ WiFi connection failed! Running without network.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Failed!");
    display.println("Running offline");
    display.display();
  }
  
  delay(1000);
  
  // Web Server
  server.on("/", handleRoot);
  server.on("/relay", handleRelay);
  server.begin();
  
  Serial.println("\n========== SYSTEM READY ==========");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("🌍 Web: http://" + WiFi.localIP().toString());
  }
  Serial.println("🔐 API Key: " + API_KEY);
  Serial.println("==================================\n");
  
  // Initial display
  displayPage1_Currents();
  
  // Reset watchdog if enabled
  // esp_task_wdt_reset();
}

// ==================== LOOP ====================
void loop() {
  static unsigned long last_current_read = 0;
  static unsigned long last_voltage_read = 0;
  static unsigned long last_log_time = 0;
  static unsigned long last_sheet_sync = 0;
  static unsigned long last_attack_check = 0;
  static unsigned long last_loop_time = 0;
  
  // Reset watchdog
  //esp_task_wdt_reset();
  
  // Handle web server
  server.handleClient();
  
  // Handle GPS (non-blocking)
  readGPS();
  
  // Handle serial commands
  handleSerialCommands();
  
  // Read currents (every 500ms)
  if (millis() - last_current_read >= 500) {
    current_R = readCurrentRMS(CURRENT_PIN_R, CURRENT_OFFSET_R);
    current_Y = readCurrentRMS(CURRENT_PIN_Y, CURRENT_OFFSET_Y);
    current_B = readCurrentRMS(CURRENT_PIN_B, CURRENT_OFFSET_B);
    last_current_read = millis();
  }
  
  // Read voltages (every 500ms)
  if (millis() - last_voltage_read >= 500) {
    voltage_R = readVoltageRMS(VOLTAGE_R);
    voltage_Y = readVoltageRMS(VOLTAGE_Y);
    voltage_B = readVoltageRMS(VOLTAGE_B);
    last_voltage_read = millis();
  }
  
  // Calculate phase differences
  if (new_pulse_R && new_pulse_Y && new_pulse_B) {
    noInterrupts();
    unsigned long t_R = time_R, t_Y = time_Y, t_B = time_B;
    float freq = stable_frequency;
    new_pulse_R = new_pulse_Y = new_pulse_B = false;
    interrupts();
    
    if (freq > 0) {
      phase_diff_RY = calculatePhaseDiff(t_R, t_Y, freq);
      phase_diff_YB = calculatePhaseDiff(t_Y, t_B, freq);
      phase_diff_BR = calculatePhaseDiff(t_B, t_R, freq);
    }
  }
  
  // Check for cyber attacks (every second)
  if (millis() - last_attack_check >= 1000) {
    checkForAttacks();
    last_attack_check = millis();
  }
  
  // Update OLED display
  updateOLEDDisplay();
  
  // Log data every 30 seconds
  if (millis() - last_log_time >= 30000) {
    logCurrentData();
    last_log_time = millis();
    
    Serial.println("=================================");
    Serial.printf("📍 %s %s | %.4f,%.4f\n", gps_date.c_str(), gps_time.c_str(), gps_lat, gps_lng);
    Serial.printf("⚡ Currents: %.2fA %.2fA %.2fA | Angles: %.1f° %.1f° %.1f°\n", 
                  current_R, current_Y, current_B, phase_diff_RY, phase_diff_YB, phase_diff_BR);
    Serial.printf("🔌 Voltages: %.0fV %.0fV %.0fV | Freq: %.1fHz\n",
                  voltage_R, voltage_Y, voltage_B, frequency);
    Serial.printf("🛡️ Attack Status: %s\n", attack_detected ? "ATTACK DETECTED!" : "Normal");
    if (attack_detected) Serial.printf("⚠️ Attack Reason: %s\n", attack_reason.c_str());
    Serial.printf("💾 Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println("=================================");
  }
  
  // Sync to Google Sheets every 2 minutes (only if connected)
  if (WiFi.status() == WL_CONNECTED && millis() - last_sheet_sync >= 120000) {
    sendToGoogleSheets();
    last_sheet_sync = millis();
  }
  
  // System health check
  checkSystemHealth();
  
  // Small delay to prevent watchdog issues
  delay(10);
}
