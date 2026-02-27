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

// ---- Shared state (written by wifiTask, read by controlTask) ----
SemaphoreHandle_t stateMutex;

int32_t bodyOffset_i   = 0;
int32_t bodyDepth_i    = 0;
int32_t torsoYaw_i     = 0;
int32_t bodyOffset     = 0;
int32_t bodyDepth      = 0;
int32_t torsoYaw       = 0;
bool    initiated      = false;
int32_t marker_x_i     = 0;
int32_t marker_y_i     = 0;
int32_t marker_theta_i = 0;
int32_t marker_x       = 0;
int32_t marker_y       = 0;
int32_t marker_theta   = 0;

unsigned long startTime;

// ---- Task handles ----
TaskHandle_t wifiTaskHandle    = NULL;
TaskHandle_t controlTaskHandle = NULL;

// ---------------------------------------------------------------

void wifiTask(void* pvParameters) {
  esp_wifi_restore();
  vTaskDelay(pdMS_TO_TICKS(200));

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  vTaskDelay(pdMS_TO_TICKS(200));
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Trying to Connect to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  while (1) {
    Serial.println("Trying to Connect to Server...");
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      Serial.println("Connected to server!");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Unblock the control task now that connectivity is established
  xTaskNotifyGive(controlTaskHandle);

  for (;;) {
    if (!client.connected()) {
      Serial.println("Disconnected. Reconnecting...");
      client.connect(SERVER_IP, SERVER_PORT);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (client.available() < 4) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    Serial.println("Got Header");

    byte lengthBytes[4];
    client.readBytes(lengthBytes, 4);
    int32_t packetLength =
      ((int32_t)lengthBytes[0])        |
      ((int32_t)lengthBytes[1] <<  8)  |
      ((int32_t)lengthBytes[2] << 16)  |
      ((int32_t)lengthBytes[3] << 24);
    Serial.println(packetLength);

    // Wait for full payload with a timeout so we never block indefinitely
    unsigned long waitStart = millis();
    while (client.available() < packetLength) {
      if (millis() - waitStart > 500) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    #define BODY_LENGTH 12
    if (packetLength == BODY_LENGTH && client.available() >= BODY_LENGTH) {
      byte data[BODY_LENGTH];
      client.readBytes(data, BODY_LENGTH);

      int32_t newBodyOffset =
        (((int32_t)data[0])       |
         ((int32_t)data[1] <<  8) |
         ((int32_t)data[2] << 16) |
         ((int32_t)data[3] << 24));

      int32_t newBodyDepth =
        (((int32_t)data[4])       |
         ((int32_t)data[5] <<  8) |
         ((int32_t)data[6] << 16) |
         ((int32_t)data[7] << 24));

      int32_t newTorsoYaw =
        (((int32_t)data[8])        |
         ((int32_t)data[9]  <<  8) |
         ((int32_t)data[10] << 16) |
         ((int32_t)data[11] << 24));

      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (!initiated) {
          bodyOffset_i   = newBodyOffset;
          bodyDepth_i    = newBodyDepth;
          torsoYaw_i     = newTorsoYaw;
          marker_x_i     = marker_x;
          marker_y_i     = marker_y;
          marker_theta_i = marker_theta;
          initiated      = true;
        }
        bodyOffset = newBodyOffset - bodyOffset_i;
        bodyDepth  = newBodyDepth  - bodyDepth_i;
        torsoYaw   = newTorsoYaw   - torsoYaw_i;
        xSemaphoreGive(stateMutex);
      }

      Serial.print("Body Offset: "); Serial.println(bodyOffset);
      Serial.print("Body Depth: ");  Serial.println(bodyDepth);
      Serial.print("Torso Yaw: ");   Serial.println(torsoYaw);
    }
  }
}

void controlTask(void* pvParameters) {
  // Wait until wifiTask has connected to WiFi and server
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  const TickType_t period   = pdMS_TO_TICKS(20); // 50 Hz
  TickType_t       lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

#if USE_EKF
    checkFiducial();
#endif

    int m1, m2, m3;

#if DEMO_MODE
    float t = (millis() - startTime) / 1000.0f;
    float localTargetX = DEMO_RADIUS * cos(DEMO_ANGULAR_SPEED * t);
    float localTargetY = DEMO_RADIUS * sin(DEMO_ANGULAR_SPEED * t);
    trackUser(st, localTargetX, localTargetY, m1, m2, m3);
#else
    float localTargetX  = 0;
    float localTargetY  = 0;
    bool  localInitiated = false;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      localInitiated = initiated;
      localTargetX   = (float)bodyDepth  / 1000.0f;
      localTargetY   = (float)bodyOffset / 1000.0f;
      xSemaphoreGive(stateMutex);
    }

    if (localInitiated) {
      trackUser(st, localTargetX, localTargetY, m1, m2, m3);
    }
#endif
  }
}

// ---------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  stateMutex = xSemaphoreCreateMutex();

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

  // controlTask must be created first so its handle is valid when wifiTask notifies it
  xTaskCreatePinnedToCore(controlTask, "controlTask", 4096, NULL, 2, &controlTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiTask,    "wifiTask",    8192, NULL, 1, &wifiTaskHandle,    0);
}

void loop() {
  vTaskDelete(NULL); // Arduino loop task not needed
}
