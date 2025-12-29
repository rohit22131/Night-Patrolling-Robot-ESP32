/**
 * ESP32-S3 Patrol Robot - FINAL UPDATED VERSION
 * Features:
 * 1. PWM Buzzer Drive (Passive & Active)
 * 2. Startup Sound Test
 * 3. 10-Second Alarm Duration
 * 4. GSM & Smart Obstacle Avoidance
 * 5. Firebase Realtime DB v1.x (FIXED)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <time.h>
#include <ESP32Servo.h>
#include <driver/i2s.h>
#include <TinyGPSPlus.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// --- USER CONFIGURATION ---
#define WIFI_SSID "realme 6"
#define WIFI_PASSWORD "rohit@123"
#define API_KEY "AIzaSyDRpeeM5RMKPW1CCQao9T1bTPjJ9jwlDM0"
#define DATABASE_URL "https://night-patrolling-robot-ca1c9-default-rtdb.asia-southeast1.firebasedatabase.app"

#define INVERT_MOTOR_B true
#define ADMIN_NUMBER "+919506170955"

// --- THRESHOLDS ---
#define NOISE_THRESHOLD 200
#define ACCEL_THRESHOLD 15.0
#define ALARM_DURATION 10000

// --- PINS ---
#define AIN1 4
#define AIN2 5
#define PWMA 6
#define BIN1 7
#define BIN2 8
#define PWMB 9
#define STBY 10

#define TRIG_PIN 11
#define ECHO_PIN 12
#define SERVO_PIN 21
#define BUZZER_PIN 42

#define GPS_RX 18
#define GPS_TX 17

#define SDA_PIN 1
#define SCL_PIN 2

#define MODEM_TX 48
#define MODEM_RX 47
#define MODEM_RST 46

#define I2S_SCK 16
#define I2S_WS 15
#define I2S_SD 14
#define I2S_PORT I2S_NUM_0

// --- OBJECTS ---
FirebaseAuth auth;
FirebaseConfig config;
FirebaseData fbDO;
FirebaseData fbUpload;
Adafruit_MPU6050 mpu;
Servo scanServo;

// --- VARIABLES ---
String currentMode = "manual";
String currentCommand = "stop";
bool mpuConnected = false;
unsigned long lastTelemetryTime = 0;
int distance = 0;
int noiseLevel = 0;

// Alarm Vars
bool alarmActive = false;
unsigned long alarmStartTime = 0;
bool theftAlertTriggered = false;

// --- TIMERS ---
unsigned long lastGPSSend = 0;
const unsigned long GPS_INTERVAL = 3000;

// --- BUZZER ---
void buzzOn() { ledcWriteTone(2, 2000); }
void buzzOff() { ledcWrite(2, 0); }

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);
  ledcAttachPin(BUZZER_PIN, 2);
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MODEM_RST, OUTPUT); digitalWrite(MODEM_RST, HIGH);

  scanServo.attach(SERVO_PIN);
  scanServo.write(90);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (mpu.begin()) {
    mpuConnected = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  }

  Serial1.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);

  buzzOn(); delay(200); buzzOff();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  delay(2000);

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// --- LOOP ---
void loop() {
  handleFirebase();
  handleAlarm();
  handleSensorsAndTelemetry();
  HandleGPS();
}

// --- FIREBASE ---
void handleFirebase() {
  if (Firebase.RTDB.getString(&fbDO, "/robot/control/mode")) {
    if (fbDO.stringData() != currentMode) {
      currentMode = fbDO.stringData();
      stopBot();
    }
  }

  if (currentMode == "manual") {
    if (Firebase.RTDB.getString(&fbDO, "/robot/control/command")) {
      if (fbDO.stringData() != currentCommand) {
        currentCommand = fbDO.stringData();
        executeMotorCommand(currentCommand);
      }
    }
  } else runSmartAutoPilot();
}

// --- ALARM ---
void handleAlarm() {
  if (alarmActive && millis() - alarmStartTime > ALARM_DURATION) {
    buzzOff();
    alarmActive = false;
    theftAlertTriggered = false;
  }
}

// --- TELEMETRY ---
void handleSensorsAndTelemetry() {
  if (millis() - lastTelemetryTime > 200) {
    readSensorsAndCheckAlerts();
    static unsigned long uploadTimer = 0;
    if (millis() - uploadTimer > 2000) {
      uploadTelemetry();
      uploadTimer = millis();
    }
    lastTelemetryTime = millis();
  }
}

// --- SENSORS ---
void measureDistance() {
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 25000);
  distance = d == 0 ? 999 : d * 0.034 / 2;
}

void measureNoise() {
  size_t bytes_read; int32_t sample; long sumSq = 0;
  for (int i = 0; i < 256; i++) {
    i2s_read(I2S_PORT, &sample, sizeof(sample), &bytes_read, portMAX_DELAY);
    int32_t s = sample >> 12;
    sumSq += (long)s * s;
  }
  noiseLevel = sqrt(sumSq / 256.0);
}

// --- ALERT CHECK ---
void readSensorsAndCheckAlerts() {
  measureDistance();
  measureNoise();

  if (noiseLevel > NOISE_THRESHOLD && !alarmActive) {
    alarmActive = true;
    alarmStartTime = millis();
    buzzOn();
    sendSMS("SUSPICIOUS SOUND DETECTED");
  }

  if (mpuConnected) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      float total = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
      if (total > ACCEL_THRESHOLD && !alarmActive) {
        theftAlertTriggered = true;
        alarmActive = true;
        alarmStartTime = millis();
        buzzOn();
        sendSMS("POSSIBLE THEFT");
      }
    }
  }
}

// --- SMS ---
void sendSMS(String type) {
  Serial1.println("AT+CMGF=1"); delay(100);
  Serial1.print("AT+CMGS=\""); Serial1.print(ADMIN_NUMBER); Serial1.println("\"");
  delay(100);
  Serial1.print("🚨 ALERT: "); Serial1.print(type);
  delay(100); Serial1.write(26);
}

// --- GPS ---
void HandleGPS() {
  while (GPS_Serial.available()) gps.encode(GPS_Serial.read());

  if (gps.location.isValid() && millis() - lastGPSSend > GPS_INTERVAL) {
    Firebase.RTDB.setDouble(&fbDO, "/robot/location/lat", gps.location.lat());
    Firebase.RTDB.setDouble(&fbDO, "/robot/location/lng", gps.location.lng());
    time_t now;
    time(&now);
    Firebase.RTDB.setInt(&fbDO, "/robot/lastSeen", (unsigned long)now * 1000);
    lastGPSSend = millis();
  }
}

// --- FIREBASE STATUS ---
void uploadTelemetry() {
  FirebaseJson json;
  json.set("distance", distance);
  json.set("noise", noiseLevel);
  json.set("alerts/suspicious_sound_alert", alarmActive);
  json.set("alerts/movement_theft_alert", theftAlertTriggered);
  Firebase.RTDB.updateNode(&fbUpload, "/robot/status", &json);
}

// --- MOTORS ---
void setMotorA(bool f, int s) {
  digitalWrite(AIN1, f); digitalWrite(AIN2, !f);
  ledcWrite(0, s);
}
void setMotorB(bool f, int s) {
  if (INVERT_MOTOR_B) f = !f;
  digitalWrite(BIN1, f); digitalWrite(BIN2, !f);
  ledcWrite(1, s);
}
void stopBot() { setMotorA(0, 0); setMotorB(0, 0); }
void forward() { setMotorA(1, 255); setMotorB(1, 255); }
void back() { setMotorA(0, 255); setMotorB(0, 255); }
void left() { setMotorA(0, 200); setMotorB(1, 200); }
void right() { setMotorA(1, 200); setMotorB(0, 200); }

void executeMotorCommand(String cmd) {
  if (cmd == "fwd") forward();
  else if (cmd == "bwd") back();
  else if (cmd == "left") left();
  else if (cmd == "right") right();
  else if (cmd == "horn_on") buzzOn();
  else if (cmd == "horn_off") buzzOff();
  else stopBot();
}

void runSmartAutoPilot() {
  if (currentMode != "auto") return;
  measureDistance();
  if (distance < 30) {
    stopBot(); delay(200);
    back(); delay(300);
    stopBot();
  } else forward();
}
