/**
 * water_monitoring_system.ino
 * Main ESP32 Firmware for the Intelligent IoT-Based Water Quality Monitoring
 * and Leakage Detection System.
 * 
 * Target: ESP32 Dev Module
 * Author: Avishka Constantine (Student ID: BT/EE/24/09)
 */

#include <WiFi.h>
#include <PubSubClient.h> // Ensure "PubSubClient" library is installed in Arduino IDE
#include "config.h"

// ==========================================
// GLOBALS & INSTANCES
// ==========================================
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Flow sensor variables (volatile as they are modified in Interrupt Service Routines)
volatile unsigned long pulseCountIn = 0;
volatile unsigned long pulseCountOut = 0;

// Variables for calculating flow rates
float flowRateIn = 0.0;
float flowRateOut = 0.0;
unsigned long lastFlowMillis = 0;
const unsigned long flowInterval = 1000; // Calculate flow every 1 second (1000ms)

// Leak detection tracking
int leakStreakCount = 0;
bool leakDetected = false;

// Quality threshold flags
bool acidicAlert = false;
bool alkalineAlert = false;
bool tdsAlert = false;
bool turbidityAlert = false;
bool contaminationDetected = false;

// Sensor readings
float phValue = 7.0;
float tdsValue = 100.0; // ppm
float turbidityValue = 1.2; // NTU

// Timing variables for overall control loop
unsigned long lastSensorPollMillis = 0;
const unsigned long sensorPollInterval = 5000; // Poll sensors every 5 seconds

// AI Engine alert tracking
bool aiHighRiskAlert = false;
int  aiRiskScore     = 0;
char aiRiskLevel[16] = "UNKNOWN";
char aiMessage[128]  = "AI Engine not connected";

// ==========================================
// INTERRUPT SERVICE ROUTINES (ISRs) WITH DEBOUNCE NOISE FILTER
// ==========================================
// Must run in IRAM for fast execution during pin state changes
void IRAM_ATTR pulseCounterIn() {
  static unsigned long lastPulseIn = 0;
  unsigned long now = micros();
  // Filter out high-frequency electrical noise (minimum 3ms pulse width)
  if (now - lastPulseIn > 3000) {
    pulseCountIn++;
    lastPulseIn = now;
  }
}

void IRAM_ATTR pulseCounterOut() {
  static unsigned long lastPulseOut = 0;
  unsigned long now = micros();
  // Filter out high-frequency electrical noise (minimum 3ms pulse width)
  if (now - lastPulseOut > 3000) {
    pulseCountOut++;
    lastPulseOut = now;
  }
}

// ==========================================
// 3-SOLENOID VALVE ACTUATION HELPER FUNCTIONS
// ==========================================
// Valve 3: Main Pipeline Cutoff (After Outlet Flow Sensor 2)
void setMainValveOpen() {
  if (SOLENOID_NORMALLY_CLOSED) {
    digitalWrite(PIN_VALVE_MAIN, HIGH);
  } else {
    digitalWrite(PIN_VALVE_MAIN, LOW);
  }
  Serial.println("[ACTUATION] Main Cutoff Valve 3 -> OPEN (Active Flow)");
}

void setMainValveClosed() {
  if (SOLENOID_NORMALLY_CLOSED) {
    digitalWrite(PIN_VALVE_MAIN, LOW);
  } else {
    digitalWrite(PIN_VALVE_MAIN, HIGH);
  }
  Serial.println("[ACTUATION] Main Cutoff Valve 3 -> CLOSED (Pipeline Cutoff)");
}

// Valves 1 & 2: Sensor Chamber Isolation Loop
void setSensorLoopOpen() {
  if (SOLENOID_NORMALLY_CLOSED) {
    digitalWrite(PIN_VALVE_SENSOR_IN, HIGH);
    digitalWrite(PIN_VALVE_SENSOR_OUT, HIGH);
  } else {
    digitalWrite(PIN_VALVE_SENSOR_IN, LOW);
    digitalWrite(PIN_VALVE_SENSOR_OUT, LOW);
  }
  Serial.println("[ACTUATION] Sensor Valves 1 & 2 -> OPEN (Sampling Loop Active)");
}

void setSensorLoopClosed() {
  if (SOLENOID_NORMALLY_CLOSED) {
    digitalWrite(PIN_VALVE_SENSOR_IN, LOW);
    digitalWrite(PIN_VALVE_SENSOR_OUT, LOW);
  } else {
    digitalWrite(PIN_VALVE_SENSOR_IN, HIGH);
    digitalWrite(PIN_VALVE_SENSOR_OUT, HIGH);
  }
  Serial.println("[ACTUATION] Sensor Valves 1 & 2 -> CLOSED (Chamber Isolated)");
}

// Master helpers
void setAllValvesOpen() {
  setMainValveOpen();
  setSensorLoopOpen();
}

void setAllValvesClosed() {
  setMainValveClosed();
  setSensorLoopClosed();
}

bool getMainValveClosedState() {
  if (SOLENOID_NORMALLY_CLOSED) {
    return (digitalRead(PIN_VALVE_MAIN) == LOW);
  } else {
    return (digitalRead(PIN_VALVE_MAIN) == HIGH);
  }
}

bool getSensorLoopClosedState() {
  if (SOLENOID_NORMALLY_CLOSED) {
    return (digitalRead(PIN_VALVE_SENSOR_IN) == LOW);
  } else {
    return (digitalRead(PIN_VALVE_SENSOR_IN) == HIGH);
  }
}

// Backwards compatibility aliases
void setValveOpen() { setAllValvesOpen(); }
void setValveClosed() { setMainValveClosed(); }
bool getValveClosedState() { return getMainValveClosedState(); }

// ==========================================
// SYSTEM SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n==========================================");
  Serial.println("Starting Water Monitoring & Leakage System...");
  Serial.println("Student: Avishka Constantine (BT/EE/24/09)");
  Serial.println("==========================================");

  // Configure Flow Sensors and Actuator Pins
  pinMode(PIN_FLOW_IN, INPUT_PULLUP);
  pinMode(PIN_FLOW_OUT, INPUT_PULLUP);
  pinMode(PIN_VALVE_MAIN, OUTPUT);
  pinMode(PIN_VALVE_SENSOR_IN, OUTPUT);
  pinMode(PIN_VALVE_SENSOR_OUT, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  // Set default safe states: all valves open for normal fluid circulation
  setAllValvesOpen();
  digitalWrite(BUZZER_PIN, LOW); // Buzzer off
  digitalWrite(PUMP_PIN, LOW);   // Water pump initially OFF upon power up! Requires dashboard button to start.

  // Attach Interrupts for Flow Sensors (Rising Edge)
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_IN), pulseCounterIn, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_OUT), pulseCounterOut, RISING);
  
  Serial.println("Flow sensor interrupts attached.");
  Serial.println("3 Solenoid valves configured (Main Cutoff: GPIO 18, Sensor In: GPIO 21, Sensor Out: GPIO 22).");
  Serial.println("Pump initialized to OFF state on GPIO 23 (Safety Standby).");

  // Set up Wi-Fi and MQTT if not in absolute offline mode
  if (!SIMULATION_MODE) {
    setupWiFi();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setBufferSize(512); // Essential for JSON telemetry packets up to 512 bytes
    
    // Set callback to receive AI alerts & Dashboard Control commands
    mqttClient.setCallback([](char* topic, byte* payload, unsigned int length) {
      String msg = "";
      for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
      String topicStr = String(topic);

      // Check if incoming message is a dashboard control command
      if (topicStr.indexOf("control") != -1) {
        Serial.printf("\n[CONTROL COMMAND RECEIVED] Topic: %s | Payload: %s\n", topic, msg.c_str());
        
        // Pump actuation
        if (msg.indexOf("\"pump\"") != -1 || msg.indexOf("pump") != -1) {
          if (msg.indexOf("ON") != -1) {
            digitalWrite(PUMP_PIN, HIGH);
            Serial.println("[ACTUATION] Mini Water Pump -> STARTED (GPIO 23 HIGH)");
          } else if (msg.indexOf("OFF") != -1) {
            digitalWrite(PUMP_PIN, LOW);
            Serial.println("[ACTUATION] Mini Water Pump -> STOPPED (GPIO 23 LOW)");
          }
        }
        
        // Manual Solenoid Valve actuation
        if (msg.indexOf("\"valve\"") != -1 || msg.indexOf("valve") != -1) {
          if (msg.indexOf("CLOSE") != -1) {
            setMainValveClosed();
            Serial.println("[ACTUATION] Main Solenoid Valve 3 -> CLOSED (Relay Triggered)");
          } else if (msg.indexOf("OPEN") != -1) {
            setAllValvesOpen();
            Serial.println("[ACTUATION] Solenoid Valves -> OPEN (Normal Operation)");
          }
        }

        // Specific Sensor Chamber Isolation Command
        if (msg.indexOf("sensor_loop") != -1 || msg.indexOf("sensor_chamber") != -1) {
          if (msg.indexOf("CLOSE") != -1) {
            setSensorLoopClosed();
          } else if (msg.indexOf("OPEN") != -1) {
            setSensorLoopOpen();
          }
        }
        return;
      }

      // Parse AI alert message from Python engine
      // We look for the risk_score field in the JSON
      int scoreIdx = msg.indexOf("\"risk_score\":");
      if (scoreIdx != -1) {
        aiRiskScore = msg.substring(scoreIdx + 13).toInt();
        aiHighRiskAlert = (aiRiskScore >= 50);
      }

      // Extract risk level
      int levelIdx = msg.indexOf("\"risk_level\":\"");
      if (levelIdx != -1) {
        int start = levelIdx + 14;
        int end   = msg.indexOf("\"", start);
        msg.substring(start, end).toCharArray(aiRiskLevel, sizeof(aiRiskLevel));
      }

      Serial.println("\n[AI ENGINE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      Serial.printf("[AI ENGINE] Risk Score: %d/100  [%s]\n", aiRiskScore, aiRiskLevel);
      if (aiHighRiskAlert) {
        Serial.println("[AI ENGINE] ⚠ HIGH RISK — AI pre-emptive alert received!");
        // Flash buzzer briefly to distinguish from full emergency alarm
        for (int i = 0; i < 3; i++) {
          digitalWrite(BUZZER_PIN, HIGH); delay(80);
          digitalWrite(BUZZER_PIN, LOW);  delay(80);
        }
      }
      Serial.println("[AI ENGINE] ━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    });
  } else {
    Serial.println("[MODE] Running in SIMULATION MODE. Hardware sensors bypassed.");
  }
  
  lastFlowMillis = millis();
  lastSensorPollMillis = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Handle Wi-Fi and MQTT connection if not in Simulation Mode
  if (!SIMULATION_MODE) {
    if (!WiFi.isConnected()) {
      connectWiFiNonBlocking();
    } else {
      if (!mqttClient.connected()) {
        reconnectMQTT();
      }
      mqttClient.loop();
    }
  }

  // 1. DYNAMIC FLOW RATE COMPUTATION (Every 1 second)
  unsigned long currentMillis = millis();
  if (currentMillis - lastFlowMillis >= flowInterval) {
    calculateFlowRates(currentMillis - lastFlowMillis);
    lastFlowMillis = currentMillis;
  }

  // 2. WATER SENSOR POLLING & CLASSIFICATION (Every 5 seconds)
  if (currentMillis - lastSensorPollMillis >= sensorPollInterval) {
    pollAndProcessSensors();
    lastSensorPollMillis = currentMillis;
  }

  // 3. BUZZER ALARM BEATING
  // If an alert is active, pulse the buzzer locally
  if (leakDetected || contaminationDetected) {
    pulseBuzzer();
  }
}

// ==========================================
// FLOW RATE CALCULATION
// ==========================================
void calculateFlowRates(unsigned long durationMs) {
  // Disable interrupts temporarily while reading pulse counts to avoid data corruption
  noInterrupts();
  unsigned long pulsesIn = pulseCountIn;
  unsigned long pulsesOut = pulseCountOut;
  pulseCountIn = 0; // Reset counts for next interval
  pulseCountOut = 0;
  interrupts();

  float timeSeconds = (float)durationMs / 1000.0;
  
  // YF-S201 formula: Flow Rate (L/min) = Frequency (Hz) / 7.5
  // Frequency = Pulses / Time
  float freqIn = (float)pulsesIn / timeSeconds;
  float freqOut = (float)pulsesOut / timeSeconds;

  if (SIMULATION_MODE) {
    // Generate simulated flow rates
    // Normal operation: Inlet and outlet match closely
    flowRateIn = 4.2 + (random(-10, 11) / 100.0); // 4.10 - 4.30 L/min
    
    // Simulate a random leak occasionally for testing
    // Every ~30 seconds, there is a 30% chance to simulate a leak
    static unsigned long lastLeakSimTime = 0;
    static bool simLeakActive = false;
    
    if (millis() - lastLeakSimTime > 30000) {
      simLeakActive = (random(0, 10) < 3);
      lastLeakSimTime = millis();
      if (simLeakActive) {
        Serial.println("[SIMULATOR] Simulating physical water leakage...");
      } else {
        Serial.println("[SIMULATOR] Leakage cleared.");
      }
    }

    if (simLeakActive) {
      flowRateOut = flowRateIn - 0.85; // Leakage delta of 0.85 L/min
    } else {
      flowRateOut = flowRateIn - 0.05; // Minor calibration difference (0.05 L/min)
    }
  } else {
    // Read from actual hardware interrupts (YF-S201: Q = freq / 7.5)
    // Physical limit for YF-S201 is 30.0 L/min. Anything higher is floating pin noise.
    if (freqIn > 225.0) {
      flowRateIn = 0.0; // Filter floating pin noise
    } else {
      flowRateIn = freqIn / 7.5;
    }

    if (freqOut > 225.0) {
      flowRateOut = 0.0; // Filter floating pin noise
    } else {
      flowRateOut = freqOut / 7.5;
    }
  }

  // Calculate flow difference (inlet - outlet)
  float flowDelta = flowRateIn - flowRateOut;
  if (flowDelta < 0) flowDelta = 0; // Prevent negative delta

  // Leakage Decision Logic
  if (flowDelta > FLOW_LEAK_DELTA_LIMIT) {
    leakStreakCount++;
    Serial.printf("[LEAK CHECK] Alert! Flow Delta: %.2f L/min (Streak: %d/%d)\n", 
                  flowDelta, leakStreakCount, FLOW_LEAK_STREAK_COUNT);
  } else {
    // Slowly reset streak count if delta recovers
    if (leakStreakCount > 0) leakStreakCount--;
  }

  if (leakStreakCount >= FLOW_LEAK_STREAK_COUNT) {
    if (!leakDetected) {
      leakDetected = true;
      triggerEmergencyContainment("Water Leakage Detected");
    }
  } else {
    if (leakDetected && leakStreakCount == 0) {
      leakDetected = false;
      checkEmergencyRecovery();
    }
  }
}

// ==========================================
// POLL SENSORS & PROCESS ALGORITHMS
// ==========================================
void pollAndProcessSensors() {
  if (SIMULATION_MODE) {
    if (POTENTIOMETER_SIMULATION) {
      // Use potentiometer knob on GPIO 32 to simulate water quality variables
      int potVal = analogRead(POT_PIN);
      // Map potentiometer (0-4095) to different test conditions
      if (potVal < 1000) {
        // Condition A: Safe water
        phValue = 7.2;
        tdsValue = 120.0;
        turbidityValue = 1.1;
      } else if (potVal >= 1000 && potVal < 2000) {
        // Condition B: Acidic Breach
        phValue = 5.8;
        tdsValue = 150.0;
        turbidityValue = 1.5;
      } else if (potVal >= 2000 && potVal < 3000) {
        // Condition C: High TDS (fertilizer runoff)
        phValue = 7.5;
        tdsValue = 680.0;
        turbidityValue = 2.4;
      } else {
        // Condition D: Turbid/Muddy contamination
        phValue = 8.1;
        tdsValue = 350.0;
        turbidityValue = 12.8;
      }
    } else {
      // Auto-simulate fluctuations and occasional contaminations
      phValue = 7.15 + (random(-15, 16) / 100.0); // 7.00 - 7.30 pH (Safe)
      tdsValue = 110.0 + random(-5, 6);           // 105 - 115 ppm (Safe)
      turbidityValue = 1.0 + (random(0, 100) / 100.0); // 1.0 - 2.0 NTU (Safe)

      // 10% chance to simulate random quality contamination every 5 seconds
      if (random(0, 100) < 10) {
        int contaminationType = random(0, 3);
        if (contaminationType == 0) {
          phValue = 5.5; // Acidic contamination
          Serial.println("[SIMULATOR] Injecting ACIDIC chemical anomaly...");
        } else if (contaminationType == 1) {
          tdsValue = 750.0; // TDS contamination
          Serial.println("[SIMULATOR] Injecting TDS contaminant breach...");
        } else {
          turbidityValue = 15.4; // Turbidity contamination
          Serial.println("[SIMULATOR] Injecting TURBIDITY mud anomaly...");
        }
      }
    }
  } else {
    // HARDWARE SENSOR MODE
    // pH sensor is physically connected to GPIO 36.
    // Turbidity sensor not available — held at stable safe reference value.
    // TDS sensor wire needs connecting to GPIO 39.

    // --- READ REAL pH SENSOR (GPIO 36) ---
    int rawPH = analogRead(PIN_PH);
    float voltPH = ((float)rawPH / 4095.0) * 3.3;

    // Convert voltage to pH using linear calibration
    // Standard formula: pH = 7 + (midVoltage - measuredVoltage) / slope
    // PH_CAL_VOLT_7 = voltage when probe is in pH 7.0 buffer (default 2.5V)
    // Slope ≈ 0.18V per pH unit for most analog pH modules
    float phSlope = (PH_CAL_VOLT_4 - PH_CAL_VOLT_7) / 3.0;
    float rawPHValue = 7.0 + (PH_CAL_VOLT_7 - voltPH) / phSlope;

    // Clamp pH reading to physically reasonable range [4.0 - 10.0]
    // This prevents floating pin noise from causing false alarms
    if (rawPHValue >= 4.0 && rawPHValue <= 10.0) {
      phValue = rawPHValue;  // Use real sensor reading
    } else {
      phValue = 7.20;        // Fallback if probe disconnected / reading unreliable
    }

    // --- TURBIDITY: Simulated (sensor not available locally) ---
    turbidityValue = 1.20;   // Stable safe WHO-compliant reference value

    // --- READ REAL TDS SENSOR (GPIO 39 / Sensor VN) ---
    int rawTDS = analogRead(PIN_TDS);
    float voltTDS = ((float)rawTDS / 4095.0) * 3.3;

    // Convert voltage to TDS (ppm)
    if (rawTDS <= 15) {
      tdsValue = 0.0; // Probe completely disconnected or 0V
    } else {
      // Standard Keyestudio / Gravity TDS polynomial formula
      float compVolt = voltTDS;
      float calculatedTDS = (133.42 * compVolt * compVolt * compVolt 
                           - 255.86 * compVolt * compVolt 
                           + 857.39 * compVolt) * 0.5;

      // Ensure sensible reading even for low voltages (e.g. ~450 ppm per Volt)
      if (calculatedTDS < 15.0 && voltTDS > 0.02) {
        calculatedTDS = voltTDS * 450.0;
      }
      tdsValue = max(0.0f, min(calculatedTDS, 2000.0f));
    }
  }

  // Classification & Threshold Evaluation
  // Real pH sensor active. Turbidity simulated. TDS in demo cycle.
  acidicAlert    = (phValue < PH_MIN_SAFE);    // pH < 6.5 → acidic alarm
  alkalineAlert  = (phValue > PH_MAX_SAFE);    // pH > 8.5 → alkaline alarm
  tdsAlert       = (tdsValue > TDS_MAX_SAFE);  // TDS > 500 ppm → contamination
  turbidityAlert = false;                       // Turbidity sensor not wired

  contaminationDetected = (acidicAlert || alkalineAlert || tdsAlert || turbidityAlert);

  // Print diagnostics to Serial Monitor
  printDiagnosticReport();

  // Action loop on chemical contamination
  if (contaminationDetected) {
    triggerEmergencyContainment("Chemical Contamination Detected");
  } else {
    checkEmergencyRecovery();
  }

  // Upload data to MQTT broker
  publishTelemetry();
}

// ==========================================
// ACTUATION & CONTAINMENT COMMANDS
// ==========================================
void triggerEmergencyContainment(const char* reason) {
  Serial.printf("\n>>> !!! EMERGENCY CONTAINMENT TRIGGERED: %s !!! <<<\n", reason);
  
  // Close the Solenoid Valve
  setValveClosed();
  
  // Pulse the buzzer briefly for alarm alert (non-continuous)
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delay(100);
    digitalWrite(BUZZER_PIN, LOW);  delay(100);
  }
  
  // Send immediate warning telemetry message
  if (WiFi.isConnected() && mqttClient.connected()) {
    char alertMsg[128];
    snprintf(alertMsg, sizeof(alertMsg), "ALERT: Valve closed! Reason: %s. pH: %.2f, TDS: %.1f, NTU: %.1f", 
             reason, phValue, tdsValue, turbidityValue);
    mqttClient.publish(MQTT_TOPIC_ALERTS, alertMsg);
  }
}

void checkEmergencyRecovery() {
  // If BOTH safety checks are now healthy, open the valve back up
  if (!leakDetected && !contaminationDetected) {
    if (getValveClosedState()) {
      Serial.println("\n>>> SYSTEM RESTORED: All quality parameters and leak checks are SAFE. <<<");
      setValveOpen();
      digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
      
      if (WiFi.isConnected() && mqttClient.connected()) {
        mqttClient.publish(MQTT_TOPIC_ALERTS, "RECOVERY: System restored. Solenoid Valve opened.");
      }
    }
  }
}

void pulseBuzzer() {
  // Simple non-blocking buzzer pulse
  static unsigned long lastBuzzerMillis = 0;
  static bool buzzerState = false;
  unsigned long cur = millis();
  
  if (cur - lastBuzzerMillis >= 500) { // Beep every 500ms
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    lastBuzzerMillis = cur;
  }
}

// ==========================================
// DIAGNOSTIC REPORTS
// ==========================================
void printDiagnosticReport() {
  Serial.println("\n=== SYSTEM STATUS REPORT ===");
  Serial.printf("Time Active: %lu seconds\n", millis() / 1000);
  Serial.printf("Inlet Flow  : %.2f L/min\n", flowRateIn);
  Serial.printf("Outlet Flow : %.2f L/min\n", flowRateOut);
  Serial.printf("Flow Delta  : %.2f L/min %s\n", 
                abs(flowRateIn - flowRateOut), 
                (leakDetected ? "[LEAK DETECTED!]" : "[OK]"));
  
  Serial.printf("Water pH    : %.2f %s\n", phValue, 
                (acidicAlert ? "[ACIDIC ANOMALY!]" : (alkalineAlert ? "[ALKALINE ANOMALY!]" : "[SAFE]")));
  
  Serial.printf("Water TDS   : %.1f ppm %s [Raw ADC: %d | Voltage: %.2fV]\n", tdsValue, 
                (tdsAlert ? "[HIGH DISSOLVED SOLIDS!]" : "[SAFE]"), analogRead(PIN_TDS), ((float)analogRead(PIN_TDS) / 4095.0) * 3.3);
  
  Serial.printf("Turbidity   : %.2f NTU %s\n", turbidityValue, 
                (turbidityAlert ? "[MUDDY/DIRTY WATER!]" : "[SAFE]"));
  
  Serial.printf("Main Valve 3: %s (Cutoff after Sensor 2)\n", 
                (getMainValveClosedState() ? "CLOSED [EMERGENCY CUTOFF]" : "OPEN [FLOW ACTIVE]"));
  Serial.printf("Sensor Loop : %s (Valves 1 & 2)\n", 
                (getSensorLoopClosedState() ? "CLOSED [CHAMBER ISOLATED]" : "OPEN [ACTIVE SAMPLING]"));
  Serial.println("=============================");
}

// ==========================================
// TELEMETRY UPLOAD (MQTT)
// ==========================================
void publishTelemetry() {
  if (!WiFi.isConnected() || !mqttClient.connected()) {
    return;
  }

  // Construct full JSON payload for AI engine consumption
  // All sensor readings + 3-valve system states included
  char payload[512];
  float flowDelta = abs(flowRateIn - flowRateOut);
  bool  mainValveClosed = getMainValveClosedState();
  bool  sensorLoopClosed = getSensorLoopClosedState();
  bool  pumpRunning = (digitalRead(PUMP_PIN) == HIGH);

  snprintf(payload, sizeof(payload),
    "{"
      "\"student_id\":\"BT/EE/24/09\","
      "\"uptime_s\":%lu,"
      "\"flow_in\":%.2f,"
      "\"flow_out\":%.2f,"
      "\"flow_delta\":%.2f,"
      "\"ph\":%.2f,"
      "\"tds\":%.1f,"
      "\"turbidity\":%.2f,"
      "\"valve_closed\":%s,"
      "\"valve_main_closed\":%s,"
      "\"sensor_loop_closed\":%s,"
      "\"pump_running\":%s,"
      "\"leak_detected\":%s,"
      "\"contamination_detected\":%s,"
      "\"ai_risk_score\":%d"
    "}",
    millis() / 1000,
    flowRateIn, flowRateOut, flowDelta,
    phValue, tdsValue, turbidityValue,
    mainValveClosed     ? "true" : "false",
    mainValveClosed     ? "true" : "false",
    sensorLoopClosed    ? "true" : "false",
    pumpRunning         ? "true" : "false",
    leakDetected        ? "true" : "false",
    contaminationDetected ? "true" : "false",
    aiRiskScore
  );

  Serial.print("[MQTT] Publishing telemetry packet... ");
  if (mqttClient.publish(MQTT_TOPIC_TELEMETRY, payload)) {
    Serial.println("SUCCESS!");
  } else {
    Serial.println("FAILED! Check MQTT connection/buffer.");
  }
}


// ==========================================
// NETWORKING FUNCTIONS (WIFI & MQTT)
// ==========================================
void setupWiFi() {
  delay(10);
  Serial.print("\n[Wi-Fi] Connecting to: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Wait up to 10 seconds for initial connection
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[Wi-Fi] Connected successfully!");
    Serial.print("[Wi-Fi] ESP32 IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[Wi-Fi] Initial connection failed. Will retry in background.");
  }
}

void connectWiFiNonBlocking() {
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 10000) { // Check every 10 seconds
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[Wi-Fi] Reconnecting to Wi-Fi...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

void reconnectMQTT() {
  static unsigned long lastMqttRetry = 0;
  if (millis() - lastMqttRetry > 5000) { // Retry connection every 5 seconds
    lastMqttRetry = millis();
    Serial.print("[MQTT] Attempting connection to broker...");
    
    // Connect to HiveMQ Public Broker with unique client ID to avoid collision
    String clientId = "ESP32_WaterNode_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" Connected successfully!");
      // Subscribe to AI engine alert topic to receive pre-emptive AI warnings
      mqttClient.subscribe("water/network/ai_alerts");
      mqttClient.subscribe(MQTT_TOPIC_CONTROL);
      Serial.println("[MQTT] Subscribed to AI alerts and Dashboard Control topics.");
      // Send a boot notification
      mqttClient.publish(MQTT_TOPIC_ALERTS, "ESP32 water quality node activated. AI layer active.");
    } else {
      Serial.print(" Failed! rc=");
      Serial.print(mqttClient.state());
      Serial.println(" (Will retry in 5 seconds)");
    }
  }
}
