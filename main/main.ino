#include <math.h>
#include <SCServo.h>
SMS_STS st;

// ESP32 pins used for communicating with motor drivers
#define S_RXD 20  
#define S_TXD 21  

const float WHEEL_RADIUS = 0.05; // meters
const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;

// Wheel angles (rad)
const float W1_ANGLE = PI / 2;       // 90°
const float W2_ANGLE = 7 * PI / 6;   // 210°
const float W3_ANGLE = 11 * PI / 6;  // 330°

u8 SERVO_IDS[3] = {1, 2, 3};

// stores global state of the motors
s16 posRead[3];
u16 speedRead[3];

// Centre-to-wheel distance, will be needed for rotation
// const float L = 0.12; // example meters

int16_t angle_to_tick(int16_t angle)
{
  // 4096 ticks = 360 degrees
  float ang_float = (float)angle;
  return (int16_t)((ang_float/360.0) * 4096.0);
}

// rotates one servo (absolute or relative depending on the mode)
// blocks until motor is finished
void rotateServo(int servoID, float degrees, uint32_t timeoutMs = 8000)
{
  st.WritePosEx(servoID, angle_to_tick(degrees), 10000, 200);

  // block until finished
  uint32_t t0 = millis();
  bool started = false;
  while (millis() - t0 < timeoutMs)
  {
    bool done = true;

    if(st.ReadMove(servoID) != 0)
    {
      done = false;
      started = true;
    }

    if(done && started) return;
  
    delay(15);
  }
}

// load a motor's state into the global variables
bool getFeedBack(u8 servoID)
{
  int r = st.FeedBack(servoID);
  if (r == -1) {
    return false;
  }

  posRead[servoID-1]     = st.ReadPos(servoID);
  speedRead[servoID-1]   = st.ReadSpeed(servoID);
  // loadRead[servoID]    = st.ReadLoad(servoID);
  // voltageRead[servoID] = (byte)st.ReadVoltage(servoID);
  // currentRead[servoID] = st.ReadCurrent(servoID);
  // temperRead[servoID]  = st.ReadTemper(servoID);
  // modeRead[servoID]    = st.ReadMode(servoID);
  return true;
}

// move three motors together, they should start and stop together
// provide the motor rotations in angles
bool MoveWheelDistanceTogether_3(u8 ids[3],
                        s16 targets[3],
                        u16 maxSpeed = 10000,
                        u8 acc   = 250,
                        uint16_t posTolTicks = 12,
                        uint16_t speedTol = 30,
                        uint32_t timeoutMs = 8000)
{
  // a minimum speed prevents stalling when dynamically computing speeds
  u16 minSpeed = 150;

  // convert all angles to step ticks.
  s16 pos[3] = { angle_to_tick(targets[0]), angle_to_tick(targets[1]), angle_to_tick(targets[2]) };

  // compute maximum distance a wheel needs to travel (all wheels should stop with this one)
  uint16_t maxDist = max(max(abs(pos[0]), abs(pos[1])), abs(pos[2]));
  if (maxDist == 0) return true; // already there

  // choose motor speeds so they all finish together
  u16 spd[3];
  u8  accs[3] = {acc, acc, acc};
  for (int i=0;i<3;i++){
    float ratio = (float)abs(pos[i]) / (float)maxDist;
    uint16_t s = (uint16_t)roundf(ratio * (float)maxSpeed);
    if (s < minSpeed) s = minSpeed;                    // avoid stalling
    spd[i] = s;
  }

  // write to all motors together
  st.SyncWritePosEx(ids, 3, pos, spd, accs);

  // block until finished
  uint32_t t0 = millis();
  bool started = false;
  while (millis() - t0 < timeoutMs) {
    bool allDone = true;

    for (int i = 0; i < 3; i++) {
      // still running
      if (st.ReadMove(ids[i]) != 0) {
        allDone = false;
        started = true;
      }
    }

    if (allDone && started) return true;

    delay(15); // poll interval
  }

  return false; // timeout
}


// displace chassis by (dx, dy) in meters.
// +dy = forward, +dx = right
void moveTranslate(float dx_m, float dy_m) {
  // wheel path lengths (meters) for pure translation
  float s1 = -sin(W1_ANGLE) * dx_m + cos(W1_ANGLE) * dy_m;
  float s2 = -sin(W2_ANGLE) * dx_m + cos(W2_ANGLE) * dy_m;
  float s3 = -sin(W3_ANGLE) * dx_m + cos(W3_ANGLE) * dy_m;

  // convert wheel path length to wheel rotation degrees
  float d1 = (s1 / WHEEL_CIRCUMFERENCE) * 360.0f;
  float d2 = (s2 / WHEEL_CIRCUMFERENCE) * 360.0f;
  float d3 = (s3 / WHEEL_CIRCUMFERENCE) * 360.0f;
  int16_t tgt[3] = { d1, d2, d3 };

  bool ok = MoveWheelDistanceTogether_3(SERVO_IDS, tgt);
}

// motor test - can be helpful
void motorTest()
{
  rotateServo(1, 180);
  delay(2000);
  rotateServo(2, 180);
  delay(2000);
  rotateServo(3, 180);
  delay(2000);
}

// set servo to relative or absolute mode
void setRelativeControlMode(int servoID, bool on)
{
  u8 code = on ? 3 : 0;
  st.unLockEprom(servoID);
  st.writeByte(servoID, SMS_STS_MODE, code);
  st.writeWord(servoID, 11, 0);
  st.LockEprom(servoID);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // setup motor serial
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);

  // set motors to relative mode
  setRelativeControlMode(SERVO_IDS[0], true);
  setRelativeControlMode(SERVO_IDS[1], true);
  setRelativeControlMode(SERVO_IDS[2], true);

  // motorTest();

  Serial.println("Forward 30cm (dx=0, dy=0.30)...");
  moveTranslate(0.0f, 0.30f);
  Serial.println("FINISHED!");
  delay(1000);

  Serial.println("Strafe right 20cm (dx=0.20, dy=0)...");
  moveTranslate(0.20f, 0.0f);
  Serial.println("FINISHED!");
  delay(1000);

  Serial.println("Diagonal fwd-right 20cm each (dx=0.20, dy=0.20)...");
  moveTranslate(0.20f, 0.20f);
  Serial.println("FINISHED!");
  delay(1000);
}

void loop() {}



