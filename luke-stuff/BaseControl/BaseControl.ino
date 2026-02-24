#include <math.h>
#include <SCServo.h>
#include "EKF.h"
#include <WiFi.h>
#include "esp_wifi.h"

#define USE_EKF 0   // set to 1 to re-enable EKF

SMS_STS st;

const char* ssid = "TobyiPhone";
const char* password = "tobybrowne";

// const char* ssid = "flamingo";
// const char* password = "password";

int32_t bodyOffset_i = 0;
int32_t bodyDepth_i = 0;
int32_t torsoYaw_i = 0;
int32_t bodyOffset = 0;
int32_t bodyDepth = 0;
int32_t torsoYaw = 0;
bool initiated = false;

WiFiClient client;

const char* SERVER_IP = "172.20.10.3";
const int SERVER_PORT = 5003;

#define S_RXD 22
#define S_TXD 23

#define OPERATION_TIME 44
#define OPERATION_MODE 33
#define TORQUE_SWITCH 40

#define WHEEL_RADIUS 0.03
#define ROBOT_RADIUS 0.250171
#define TICKS_PER_REV 4096

const float WHEEL_CIRCUMFERENCE = 2*PI*WHEEL_RADIUS;

const float W1_ANGLE = PI/2;
const float W2_ANGLE = 7*PI/6;
const float W3_ANGLE = 11*PI/6;

EKF3 ekf;

//State Vars (kept in sync with ekf.x after every predict/update)
float x = 0;
float y = 0;
float theta = 0;

unsigned long lastFiducialTime = 0;
const unsigned long FIDUCIAL_INTERVAL_MS = 2000; // fudged: fire every 2 s

long prevEnc1 = 0;
long prevEnc2 = 0;
long prevEnc3 = 0;

//Target Placeholder

float targetX = 1;
float targetY = 0;
float targetTheta = 0;



u8 SERVO_IDS[3] = {1,2,3};
s16 posRead[3];
u16 speedRead[3];


// Writes velocities to all 3 servos
void writeVelocitySync3(SMS_STS st, u8 id1, u8 id2, u8 id3, int velocity1, int velocity2, int velocity3){
  byte ID[3];
  u8 lsbvec[3];
  u8 msbvec[3];
  ID[0] = id1;
  ID[1] = id2;
  ID[2] = id3;
  if (velocity1 > 1000) velocity1 = 1000;
  if (velocity1 < -1000) velocity1 = -1000;
  if (velocity2 > 1000) velocity2 = 1000;
  if (velocity2 < -1000) velocity2 = -1000;
  if (velocity3 > 1000) velocity3 = 1000;
  if (velocity3 < -1000) velocity3 = -1000;

  uint16_t magnitude1 = abs(velocity1);          // 10-bit magnitude
  uint16_t signBit1   = (velocity1 < 0) ? 1 : 0; // bit 10
  uint16_t magnitude2 = abs(velocity2);          // 10-bit magnitude
  uint16_t signBit2   = (velocity2 < 0) ? 1 : 0; // bit 10
  uint16_t magnitude3 = abs(velocity3);          // 10-bit magnitude
  uint16_t signBit3   = (velocity3 < 0) ? 1 : 0; // bit 10

  // Build 11-bit value: [bit10 = sign][bits9..0 = magnitude]
  uint16_t value111 = (signBit1 << 10) | magnitude1;
  uint16_t value112 = (signBit2 << 10) | magnitude2;
  uint16_t value113 = (signBit3 << 10) | magnitude3;

  // Split into bytes
  uint8_t lsb1 = value111 & 0xFF;          // bits 0–7
  uint8_t msb1 = (value111 >> 8) & 0xFF;   // bits 8–10
  uint8_t lsb2 = value112 & 0xFF;          // bits 0–7
  uint8_t msb2 = (value112 >> 8) & 0xFF;   // bits 8–10
  uint8_t lsb3 = value113 & 0xFF;          // bits 0–7
  uint8_t msb3 = (value113 >> 8) & 0xFF;   // bits 8–10

  lsbvec[0] = lsb1;
  lsbvec[1] = lsb2;
  lsbvec[2] = lsb3;

  msbvec[0] = msb1;
  msbvec[1] = msb2;
  msbvec[2] = msb3;

  // Write bytes
  st.syncWrite(ID, 3, 44, lsbvec, 1);
  st.syncWrite(ID, 3, 45, msbvec, 1);
}

// Calculate velocities to go in direction theta at speed scalar
void calculateVelocities(float theta, float scalar, int &m1, int &m2, int &m3){
  float vx = cos(theta)*scalar;
  float vy = sin(theta)*scalar;

  float w1 = 0.866*vx + 0.5*vy;
  float w2 = -vy;
  float w3 = -0.866*vx + 0.5*vy;

  float maxMag = max(abs(w1), max(abs(w2), abs(w3)));
  if(maxMag > 1.0){
    w1 /= maxMag;
    w2 /= maxMag;
    w3 /= maxMag;
  }

  m1 = -w1*500;
  m2 = -w2*500;
  m3 = -w3*500;
}


// Update position estimate from wheel movement
void updatePos(SMS_STS st){
  int enc1 = st.ReadPos(1);
  int enc2 = st.ReadPos(2);
  int enc3 = st.ReadPos(3);

  int d1 = enc1 - prevEnc1;
  int d2 = enc2 - prevEnc2;
  int d3 = enc3 - prevEnc3;

  if(d1 > TICKS_PER_REV/2){
    d1 -= TICKS_PER_REV;
  } else if(d1 < -TICKS_PER_REV/2){
    d1 += TICKS_PER_REV;
  }
  if(d2 > TICKS_PER_REV/2){
    d2 -= TICKS_PER_REV;
  } else if(d2 < -TICKS_PER_REV/2){
    d2 += TICKS_PER_REV;
  }
  if(d3 > TICKS_PER_REV/2){
    d3 -= TICKS_PER_REV;
  } else if(d3 < -TICKS_PER_REV/2){
    d3 += TICKS_PER_REV;
  }

  prevEnc1 = enc1;
  prevEnc2 = enc2;
  prevEnc3 = enc3;

  double a1 = d1*(2.0*PI/TICKS_PER_REV)*WHEEL_RADIUS;
  double a2 = d2*(2.0*PI/TICKS_PER_REV)*WHEEL_RADIUS;
  double a3 = d3*(2.0*PI/TICKS_PER_REV)*WHEEL_RADIUS;

  double a1x = a1*cos(PI/6.0);
  double a3x = -a3*cos(PI/6.0);

  double a1y = -a1*sin(PI/6.0);
  double a2y = a2;
  double a3y = -a3*sin(PI/6.0);

  double thetadir = (a1 + a2 + a3)/(3*ROBOT_RADIUS);

  double xdir = a1x + a3x;
  double ydir = -(a1y + a2y + a3y);

#if USE_EKF
  // EKF predict with odometry deltas; sync globals from EKF state
  ekf.predictOmni((float)xdir, (float)ydir, (float)thetadir);
  x     = ekf.x[0];
  y     = ekf.x[1];
  theta = ekf.x[2];
#else
  x     += (float)xdir;
  y     += (float)ydir;
  theta += (float)thetadir;
#endif

  // Serial.print("Enc1: ");
  // Serial.print(enc1);
  // Serial.print(" Enc2: ");
  // Serial.print(enc2);
  // Serial.print(" Enc3: ");
  // Serial.println(enc3);

  // Serial.print("X: ");
  // Serial.print(x, 6);
  // Serial.print(" Y: ");
  // Serial.print(y, 6);
  // Serial.print(" Theta: ");
  // Serial.println(theta, 6);



  // Serial.print("X: ");
  // Serial.print(x, 6);
  // Serial.print(" Y: ");
  // Serial.print(y, 6);
  // Serial.print(" Theta: ");
  // Serial.println(theta, 6);


}

// Move in direction theta at speed scalar
void move(SMS_STS st, float theta, float scalar, int &m1, int &m2, int &m3){
  calculateVelocities(theta, scalar, m1, m2, m3);
  writeVelocitySync3(st, 1, 2, 3, m1, m2, m3);
}

// Move to position x and y
void moveTo(SMS_STS &st, float targetX, float targetY, int &m1, int &m2, int &m3){
  const float ERROR_MARGIN = 0.01;
  const float KP = 1.5;
  const float MAX_SPEED = 1.0;
  const float BRAKE_ACCELERATION = 10.0;

  while(true){
    updatePos(st);

    float errorX = targetX - x;
    float errorY = targetY - y;
    float distance = sqrt(errorX*errorX + errorY*errorY);
  
    Serial.print("errorX: ");
    Serial.print(errorX, 6);
    Serial.print(" errorY: ");
    Serial.print(errorY, 6);
    Serial.print(" Distance: ");
    Serial.println(distance, 6);

    if(distance < ERROR_MARGIN){
      move(st, 0, 0, m1, m2, m3);
      break;
    }

    float errorX_robot = cos(theta)*errorX + sin(theta)*errorY;
    float errorY_robot = -sin(theta)*errorX + cos(theta)*errorY;

    float moveTheta = atan2(errorY_robot, errorX_robot);
    float scalar = sqrt(2.0 * BRAKE_ACCELERATION * distance);

    if(scalar > MAX_SPEED){
      scalar = MAX_SPEED;
    } else if(scalar < 0.08){
      scalar = 0.08;
    }

    move(st, moveTheta, scalar, m1, m2, m3);

    delay(10);
  }
}

// Second move to x y function to track a user (non-blocking)
void trackUser(SMS_STS &st, float targetX, float targetY, int &m1, int &m2, int &m3){
  const float KP = 10;
  const float KD = 0.8;
  const float MAX_SPEED = 1.0;

  static float prevErrorX = 0;
  static float prevErrorY = 0;
  updatePos(st);
  float errorX = targetX - x;
  float errorY = targetY - y;

  float dErrorX = errorX - prevErrorX;
  float dErrorY = errorY - prevErrorY;

  prevErrorX = errorX;
  prevErrorY = errorY;

  float vx_world = KP*errorX + KD*dErrorX;
  float vy_world = KP*errorY + KD*dErrorY;

  float vx_robot = cos(theta)*vx_world + sin(theta)*vy_world;
  float vy_robot = -sin(theta)*vx_world + cos(theta)*vy_world;

  float speed = sqrt(vx_robot*vx_robot + vy_robot*vy_robot);

  if (speed > MAX_SPEED){
    vx_robot *= MAX_SPEED/speed;
    vy_robot *= MAX_SPEED/speed;
    speed = MAX_SPEED;
  }

  float moveTheta = atan2(vy_robot, vx_robot);

  move(st, moveTheta, speed, m1, m2, m3);
}

// Initial set position at (0,0) and reset EKF
void calibratePos(){
  x = 0;
  y = 0;
  theta = 0;
  ekf.x[0] = 0; ekf.x[1] = 0; ekf.x[2] = 0;
}

// Called periodically; in production replace the fudged values with real
// fiducial measurements received over Serial / ROS / etc.
void checkFiducial()
{
  unsigned long now = millis();
  if (now - lastFiducialTime < FIDUCIAL_INTERVAL_MS) return;
  lastFiducialTime = now;

  // --- FUDGED measurement: true position + small Gaussian-ish noise ---
  float noiseX     = ((float)random(-40, 40)) / 1000.0f;  // ±40 mm
  float noiseY     = ((float)random(-40, 40)) / 1000.0f;
  float noiseTheta = ((float)random(-50, 50)) / 1000.0f;  // ±0.05 rad
  float measX     = x + noiseX;
  float measY     = y + noiseY;
  float measTheta = theta + noiseTheta;

  bool accepted = ekf.updateFiducial(measX, measY, measTheta);
  if (accepted) {
    x     = ekf.x[0];
    y     = ekf.x[1];
    theta = ekf.x[2];
    Serial.print("[Fiducial] meas=( ");
    Serial.print(measX, 3); Serial.print(", ");
    Serial.print(measY, 3); Serial.print(")  EKF=( ");
    Serial.print(x, 4); Serial.print(", ");
    Serial.print(y, 4); Serial.println(")");
  }
}

float centerX = 0.0;
float centerY = 0.0;
float radius = 0.5;
float angularSpeed = PI/2;

unsigned long startTime;

void scanWiFiNetworks() {
  Serial.println("\n--- Scanning WiFi Networks ---");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);

  int n = WiFi.scanNetworks(false, true);  // sync scan, show hidden

  if (n == 0) {
    Serial.println("No networks found.");
    return;
  }

  Serial.printf("Found %d networks:\n\n", n);

  for (int i = 0; i < n; ++i) {
    String enc;

    switch (WiFi.encryptionType(i)) {
      case WIFI_AUTH_OPEN:            enc = "OPEN"; break;
      case WIFI_AUTH_WEP:             enc = "WEP"; break;
      case WIFI_AUTH_WPA_PSK:         enc = "WPA_PSK"; break;
      case WIFI_AUTH_WPA2_PSK:        enc = "WPA2_PSK"; break;
      case WIFI_AUTH_WPA_WPA2_PSK:    enc = "WPA_WPA2_PSK"; break;
      case WIFI_AUTH_WPA2_ENTERPRISE: enc = "WPA2_ENTERPRISE"; break;
      case WIFI_AUTH_WPA3_PSK:        enc = "WPA3_PSK"; break;
      case WIFI_AUTH_WPA2_WPA3_PSK:   enc = "WPA2_WPA3_PSK"; break;
      default:                        enc = "UNKNOWN";
    }

    Serial.printf(
      "%2d) SSID: %-25s  RSSI: %4d dBm  CH: %2d  ENC: %s  BSSID: %s\n",
      i + 1,
      WiFi.SSID(i).c_str(),
      WiFi.RSSI(i),
      WiFi.channel(i),
      enc.c_str(),
      WiFi.BSSIDstr(i).c_str()
    );

    delay(10);
  }

  Serial.println("\n--- Scan Complete ---\n");
}

bool beginWithBestMatch(const char* targetSSID, const char* pass) {
  Serial.println("Scanning for target to lock BSSID+channel...");
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) return false;

  int best = -1;
  int bestRssi = -999;

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == targetSSID) {
      int rssi = WiFi.RSSI(i);
      if (rssi > bestRssi) { bestRssi = rssi; best = i; }
    }
  }
  if (best < 0) return false;

  int ch = WiFi.channel(best);
  uint8_t* bssid = WiFi.BSSID(best);
  Serial.printf("Locking to SSID=%s CH=%d BSSID=%s RSSI=%d\n",
                targetSSID, ch, WiFi.BSSIDstr(best).c_str(), WiFi.RSSI(best));

  WiFi.begin(targetSSID, pass, ch, bssid, true);
  return true;
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf("DISCONNECTED. reason=%d\n", info.wifi_sta_disconnected.reason);
  } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    Serial.println("CONNECTED to AP");
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.print("GOT IP: ");
    Serial.println(WiFi.localIP());
  }
}

bool connectWpa2Compat(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);

  // 1) Kick WiFi once so the driver is initialized (prevents NOT_INIT)
  WiFi.begin(ssid, pass);
  delay(200);

  // 2) Stop any in-progress attempt so we can re-config cleanly
  WiFi.disconnect(true);
  delay(200);

  // 3) Now apply IDF-level config
  wifi_config_t cfg = {};
  strlcpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
  strlcpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password));

  cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // force WPA2
  cfg.sta.pmf_cfg.capable = true;
  cfg.sta.pmf_cfg.required = false;

  esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_config failed: %d\n", (int)err);
    return false;
  }

  err = esp_wifi_connect();
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_connect failed: %d\n", (int)err);
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);

  esp_wifi_restore();   // clears wifi config stored in flash (NVS)
  delay(200);

  // scanWiFiNetworks();

  WiFi.onEvent(WiFiEvent);

  // WiFi.mode(WIFI_STA);
  // WiFi.setSleep(false);         // important for some hotspots
  // WiFi.disconnect(true);
  // delay(500);

  // // Connect to Socket
  // if (!beginWithBestMatch(ssid, password)) {
  //   Serial.println("Target SSID not found in scan results.");
  //   return;
  // }
  // WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(200);

  // WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK); // Arduino-level WPA2 minimum
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Trying to Connect to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  while (1)
  {
    Serial.println("Trying to Connect to Server...");
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      Serial.println("Connected to server!");
      break;
    } else {
      delay(1000);
    }
  }
  
  // Finish Socket Setup

  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
  for(int i=1; i<4; i++){
    st.unLockEprom(i);
    st.writeByte(i, OPERATION_MODE, 2);
    st.LockEprom(i);
    st.CalibrationOfs(i);
  }

  st.EnableTorque(1, 1);
  st.EnableTorque(2, 1);
  st.EnableTorque(3, 1);

  // --- EKF init ---
  // Process noise Q: how much we trust odometry (per control cycle)
  memset(ekf.Q, 0, sizeof(ekf.Q));
  ekf.Q[0][0] = 1e-4f;   // x
  ekf.Q[1][1] = 1e-4f;   // y
  ekf.Q[2][2] = 1e-5f;   // theta

  // Measurement noise R: fiducial marker uncertainty
  memset(ekf.R, 0, sizeof(ekf.R));
  ekf.R[0][0] = 0.01f;   // x  (±100 mm 1-sigma)
  ekf.R[1][1] = 0.01f;   // y
  ekf.R[2][2] = 0.005f;  // theta (±~4 deg 1-sigma)

  // Initial covariance P (start confident; calibratePos sets x=y=th=0)
  memset(ekf.P, 0, sizeof(ekf.P));
  ekf.P[0][0] = 1e-6f;
  ekf.P[1][1] = 1e-6f;
  ekf.P[2][2] = 1e-6f;

  ekf.b = 2.0f * ROBOT_RADIUS; // not used by predictOmni, kept for predict()

  calibratePos();

  startTime = millis();

  // int m1, m2, m3;
  // move(st, PI/2, 1, m1, m2, m3);
}


void loop() {
  if (client.connected()) {
    // Check if data available (packet format: 4-byte length header + payload)
    if (client.available() >= 4) {
      Serial.println("Got Header");
      // Read packet length
      byte lengthBytes[4];
      client.readBytes(lengthBytes, 4);

      int32_t packetLength =
        ((int32_t)lengthBytes[0]) |
        ((int32_t)lengthBytes[1] << 8) |
        ((int32_t)lengthBytes[2] << 16) |
        ((int32_t)lengthBytes[3] << 24);
      Serial.println(packetLength);

      // Wait for full packet to arrive (12 bytes = 3 int32 values)
      if (packetLength == 12 && client.available() >= 12) {
        Serial.println("Got actual body.");
        byte data[12];
        client.readBytes(data, 12);

        // Parse the three int32 values
        bodyOffset =
            (((int32_t)data[0]) |
            ((int32_t)data[1] << 8) |
            ((int32_t)data[2] << 16) |
            ((int32_t)data[3] << 24)) - bodyOffset_i;

        bodyDepth =
            (((int32_t)data[4]) |
            ((int32_t)data[5] << 8) |
            ((int32_t)data[6] << 16) |
            ((int32_t)data[7] << 24)) - bodyDepth_i;

        torsoYaw =
            (((int32_t)data[8]) |
            ((int32_t)data[9] << 8) |
            ((int32_t)data[10] << 16) |
            ((int32_t)data[11] << 24)) - torsoYaw_i;

        Serial.print("Body Offset: ");
        Serial.println(bodyOffset);

        Serial.print("Body Depth: ");
        Serial.println(bodyDepth);

        Serial.print("Torso Yaw: ");
        Serial.println(torsoYaw);
      }
    }
  }
  else {
    Serial.println("Disconnected. Reconnecting...");
    client.connect(SERVER_IP, SERVER_PORT);
    delay(1000);
  }

  if(!initiated)
  {
    bodyOffset_i = bodyOffset;
    bodyDepth_i = bodyDepth;
    bodyOffset_i = bodyOffset;
  }

// #if USE_EKF
//   checkFiducial();
// #endif

  if(initiated)
  {
    int m1, m2, m3;
    float t = (millis() - startTime)/1000.0;

    // targetX = centerX + radius*cos(angularSpeed*t);
    // targetY = centerY + radius*sin(angularSpeed*t);

    targetX = bodyOffset / 1000;
    targetY = bodyDepth / 1000;

    // send the robot to an absolute x and y co-ord
    trackUser(st, targetX, targetY, m1, m2, m3);

    //moveTo(st, 0.5, 2.0, m1, m2, m3);
    delay(10);
  }
}
