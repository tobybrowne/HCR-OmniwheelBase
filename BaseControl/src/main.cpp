#include <Arduino.h>
#include <math.h>
#include <SCServo.h>
#include "config.h"
#include "wifi_manager.h"
#include "motion.h"

// ---- Servo ----
SMS_STS st;

// ---- WiFi / Server ----
const char* ssid        = "TobyiPhone";
const char* password    = "tobybrowne";
const char* SERVER_IP   = "172.20.10.3";
const int   SERVER_PORT = 5003;
WiFiClient  client;

// ---- Pose (shared with odometry / motion via extern) ----
float x     = 0;
float y     = 0;
float theta = 0;

long prevEnc1 = 0;
long prevEnc2 = 0;
long prevEnc3 = 0;

unsigned long lastFiducialTime = 0;

// ---- EKF ----
EKF3 ekf;

// ---- Application state ----
int32_t bodyOffset_i = 0;
int32_t bodyDepth_i  = 0;
int32_t torsoYaw_i   = 0;
int32_t bodyOffset   = 0;
int32_t bodyDepth    = 0;
int32_t torsoYaw     = 0;
bool    initiated    = false;
int32_t marker_x_i     = 0;
int32_t marker_y_i     = 0;
int32_t marker_theta_i     = 0;
int32_t marker_x     = 0;
int32_t marker_y     = 0;
int32_t marker_theta     = 0;


float targetX     = 1;
float targetY     = 0;
float targetTheta = 0;

unsigned long startTime;

// ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  esp_wifi_restore();
  delay(200);

  WiFi.onEvent(WiFiEvent);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Trying to Connect to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  while (1) {
    Serial.println("Trying to Connect to Server...");
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      Serial.println("Connected to server!");
      break;
    }
    delay(1000);
  }

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);

  for (int i = 1; i < 4; i++) {
    st.unLockEprom(i);
    st.writeByte(i, OPERATION_MODE, 2);
    st.LockEprom(i);
    st.CalibrationOfs(i);
  }

  st.EnableTorque(1, 1);
  st.EnableTorque(2, 1);
  st.EnableTorque(3, 1);

  // EKF init
  memset(ekf.Q, 0, sizeof(ekf.Q));
  ekf.Q[0][0] = 1e-4f; ekf.Q[1][1] = 1e-4f; ekf.Q[2][2] = 1e-5f;

  memset(ekf.R, 0, sizeof(ekf.R));
  ekf.R[0][0] = 0.01f; ekf.R[1][1] = 0.01f; ekf.R[2][2] = 0.005f;

  memset(ekf.P, 0, sizeof(ekf.P));
  ekf.P[0][0] = 1e-6f; ekf.P[1][1] = 1e-6f; ekf.P[2][2] = 1e-6f;

  ekf.b = 2.0f * ROBOT_RADIUS;

  calibratePos();

  startTime = millis();
}

void loop() {
  if (client.connected()) {
    if (client.available() >= 4) {
      Serial.println("Got Header");

      byte lengthBytes[4];
      client.readBytes(lengthBytes, 4);
      int32_t packetLength =
        ((int32_t)lengthBytes[0])        |
        ((int32_t)lengthBytes[1] <<  8)  |
        ((int32_t)lengthBytes[2] << 16)  |
        ((int32_t)lengthBytes[3] << 24);
      Serial.println(packetLength);

      while(client.available() < packetLength) {
        delay(10);
      }

      #define BODY_LENGTH 12
      if (packetLength == BODY_LENGTH && client.available() >= BODY_LENGTH) {
        byte data[BODY_LENGTH];
        client.readBytes(data, BODY_LENGTH);

        bodyOffset =
          (((int32_t)data[0])       |
           ((int32_t)data[1] <<  8) |
           ((int32_t)data[2] << 16) |
           ((int32_t)data[3] << 24)) - bodyOffset_i;

        bodyDepth =
          (((int32_t)data[4])       |
           ((int32_t)data[5] <<  8) |
           ((int32_t)data[6] << 16) |
           ((int32_t)data[7] << 24)) - bodyDepth_i;

        torsoYaw =
          (((int32_t)data[8])        |
           ((int32_t)data[9]  <<  8) |
           ((int32_t)data[10] << 16) |
           ((int32_t)data[11] << 24)) - torsoYaw_i;

        // marker_x =
        //   (((int32_t)data[12])       |
        //    ((int32_t)data[13] <<  8) |
        //    ((int32_t)data[14] << 16) |
        //    ((int32_t)data[15] << 24)) - marker_x_i;

        // marker_y =
        //   (((int32_t)data[16])       |
        //    ((int32_t)data[17] <<  8) |
        //    ((int32_t)data[18] << 16) |
        //    ((int32_t)data[19] << 24)) - marker_y_i;

        // marker_theta = 
        //   (((int32_t)data[20])       |
        //    ((int32_t)data[21] <<  8) |
        //    ((int32_t)data[22] << 16) |
        //    ((int32_t)data[23] << 24)) - marker_theta_i;

        Serial.print("Body Offset: "); Serial.println(bodyOffset);
        Serial.print("Body Depth: ");  Serial.println(bodyDepth);
        Serial.print("Torso Yaw: ");   Serial.println(torsoYaw);
        // Serial.print("Marker X: ");    Serial.println(marker_x);
        // Serial.print("Marker Y: ");    Serial.println(marker_y);
        // Serial.print("Marker Theta: "); Serial.println(marker_theta);
        // Serial.print("Marker X (init): ");    Serial.println(marker_x_i);
        // Serial.print("Marker Y (init): ");    Serial.println(marker_y_i);

        // Serial.print("Estimated X: ");    Serial.println(x);
        // Serial.print("Estimated Y: ");    Serial.println(y);
      }
    }
  } else {
    Serial.println("Disconnected. Reconnecting...");
    client.connect(SERVER_IP, SERVER_PORT);
    delay(1000);
    return;
  }

  marker_x *= -1;
  marker_y *= -1;

  if (!initiated) { 
    bodyOffset_i = bodyOffset;
    bodyDepth_i  = bodyDepth;
    torsoYaw_i   = torsoYaw;
    marker_x_i   = marker_x;
    marker_y_i   = marker_y;
    marker_theta_i   = marker_theta;

    initiated = true;
  }

#if USE_EKF
  checkFiducial();
#endif

  int m1, m2, m3;

#if DEMO_MODE
  float t = (millis() - startTime) / 1000.0f;
  targetX = DEMO_RADIUS * cos(DEMO_ANGULAR_SPEED * t);
  targetY = DEMO_RADIUS * sin(DEMO_ANGULAR_SPEED * t);
  trackUser(st, targetX, targetY, m1, m2, m3);
  delay(10);
#else
  if (initiated) {
    targetY = (float)bodyOffset / 1000;
    targetX = (float)bodyDepth  / 1000;

    // targetY = 0;
    // targetX = 0;
    trackUser(st, targetX, targetY, m1, m2, m3);
    delay(10);
  }
#endif
}
