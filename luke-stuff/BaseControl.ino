#include <math.h>
#include <SCServo.h>

SMS_STS st;

#define S_RXD 20
#define S_TXD 21

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

//State Vars
float x = 0;
float y = 0;
float theta = 0;

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

  m1 = -w1*1000;
  m2 = -w2*1000;
  m3 = -w3*1000;
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

  x += xdir;
  y += ydir;
  theta += thetadir;

  // Serial.print("Enc1: ");
  // Serial.print(enc1);
  // Serial.print(" Enc2: ");
  // Serial.print(enc2);
  // Serial.print(" Enc3: ");
  // Serial.println(enc3);

  Serial.print("X: ");
  Serial.print(x, 6);
  Serial.print(" Y: ");
  Serial.print(y, 6);
  Serial.print(" Theta: ");
  Serial.println(theta, 6);



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

// Initial set position at (0,0)
void calibratePos(){
  x = 0;
  y = 0;
  theta = 0;
}

float centerX = 0.0;
float centerY = 0.0;
float radius = 0.5;
float angularSpeed = PI/2;

unsigned long startTime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  calibratePos();

  startTime = millis();

  // int m1, m2, m3;
  // move(st, PI/2, 1, m1, m2, m3);
}


void loop() {

  int m1, m2, m3;
  float t = (millis() - startTime)/1000.0;

  targetX = centerX + radius*cos(angularSpeed*t);
  targetY = centerY + radius*sin(angularSpeed*t);

  trackUser(st, targetX, targetY, m1, m2, m3);

  //moveTo(st, 0.5, 2.0, m1, m2, m3);
  delay(10);

  // move(st, PI/2.0, 1, m1, m2, m3);
  // for(int i=0; i<50; i++){
  //   updatePos(st);
  //   delay(20);
  // }
  // move(st, PI, 1, m1, m2, m3);
  // for(int i=0; i<50; i++){
  //   updatePos(st);
  //   delay(20);
  // }
  // move(st, 3.0*PI/2.0, 1, m1, m2, m3);
  // for(int i=0; i<50; i++){
  //   updatePos(st);
  //   delay(20);
  // }
  // move(st, 0, 1, m1, m2, m3);
  // for(int i=0; i<50; i++){
  //   updatePos(st);
  //   delay(20);
  // }
  // move(st, 0, 0, m1, m2, m3);
  // delay(10000);

  // writeVelocitySync3(st, 1, 2, 3, 500, 500, 500);
  // for(int i=0; i<200; i++){
  //   updatePos(st);
  //   delay(20);
  // }



}
