#pragma once

#include <Arduino.h>
#include <math.h>
#include <SCServo.h>
#include "config.h"
#include "odometry.h"

extern float x, y, theta;

void writeVelocitySync3(SMS_STS st, u8 id1, u8 id2, u8 id3,
                        int velocity1, int velocity2, int velocity3) {
  byte ID[3] = {id1, id2, id3};
  u8 lsbvec[3], msbvec[3];

  if (velocity1 >  1000) velocity1 =  1000;
  if (velocity1 < -1000) velocity1 = -1000;
  if (velocity2 >  1000) velocity2 =  1000;
  if (velocity2 < -1000) velocity2 = -1000;
  if (velocity3 >  1000) velocity3 =  1000;
  if (velocity3 < -1000) velocity3 = -1000;

  uint16_t v1 = ((velocity1 < 0) ? (1 << 10) : 0) | (uint16_t)abs(velocity1);
  uint16_t v2 = ((velocity2 < 0) ? (1 << 10) : 0) | (uint16_t)abs(velocity2);
  uint16_t v3 = ((velocity3 < 0) ? (1 << 10) : 0) | (uint16_t)abs(velocity3);

  lsbvec[0] = v1 & 0xFF; msbvec[0] = (v1 >> 8) & 0xFF;
  lsbvec[1] = v2 & 0xFF; msbvec[1] = (v2 >> 8) & 0xFF;
  lsbvec[2] = v3 & 0xFF; msbvec[2] = (v3 >> 8) & 0xFF;

  st.syncWrite(ID, 3, 44, lsbvec, 1);
  st.syncWrite(ID, 3, 45, msbvec, 1);
}

void calculateVelocities(float dir, float scalar, int &m1, int &m2, int &m3) {
  float vx = cos(dir) * scalar;
  float vy = sin(dir) * scalar;

  float w1 =  0.866f*vx + 0.5f*vy;
  float w2 = -vy;
  float w3 = -0.866f*vx + 0.5f*vy;

  float maxMag = max(abs(w1), max(abs(w2), abs(w3)));
  if (maxMag > 1.0f) { w1 /= maxMag; w2 /= maxMag; w3 /= maxMag; }

  m1 = (int)(-w1 * 500);
  m2 = (int)(-w2 * 500);
  m3 = (int)(-w3 * 500);
}

void move(SMS_STS st, float dir, float scalar, int &m1, int &m2, int &m3) {
  calculateVelocities(dir, scalar, m1, m2, m3);
  writeVelocitySync3(st, 1, 2, 3, m1, m2, m3);
}

void moveTo(SMS_STS &st, float targetX, float targetY, int &m1, int &m2, int &m3) {
  const float ERROR_MARGIN      = 0.01f;
  const float MAX_SPEED         = 1.0f;
  const float BRAKE_ACCELERATION = 10.0f;

  while (true) {
    updatePos(st);

    float errorX   = targetX - x;
    float errorY   = targetY - y;
    float distance = sqrt(errorX*errorX + errorY*errorY);

    Serial.print("errorX: ");   Serial.print(errorX, 6);
    Serial.print(" errorY: ");  Serial.print(errorY, 6);
    Serial.print(" Distance: "); Serial.println(distance, 6);

    if (distance < ERROR_MARGIN) {
      move(st, 0, 0, m1, m2, m3);
      break;
    }

    float errorX_robot =  cos(theta)*errorX + sin(theta)*errorY;
    float errorY_robot = -sin(theta)*errorX + cos(theta)*errorY;
    float moveDir = atan2(errorY_robot, errorX_robot);
    float scalar  = sqrt(2.0f * BRAKE_ACCELERATION * distance);

    if      (scalar > MAX_SPEED) scalar = MAX_SPEED;
    else if (scalar < 0.08f)     scalar = 0.08f;

    move(st, moveDir, scalar, m1, m2, m3);
    delay(10);
  }
}

void trackUser(SMS_STS &st, float targetX, float targetY, int &m1, int &m2, int &m3) {
  const float KP        = 10.0f;
  const float KD        = 0.8f;
  const float MAX_SPEED = 1.0f;

  static float prevErrorX = 0;
  static float prevErrorY = 0;

  updatePos(st);

  float errorX  = targetX - x;
  float errorY  = targetY - y;
  float dErrorX = errorX - prevErrorX;
  float dErrorY = errorY - prevErrorY;
  prevErrorX = errorX;
  prevErrorY = errorY;

  float vx_world =  KP*errorX + KD*dErrorX;
  float vy_world =  KP*errorY + KD*dErrorY;
  float vx_robot =  cos(theta)*vx_world + sin(theta)*vy_world;
  float vy_robot = -sin(theta)*vx_world + cos(theta)*vy_world;

  float speed = sqrt(vx_robot*vx_robot + vy_robot*vy_robot);
  if (speed > MAX_SPEED) {
    vx_robot *= MAX_SPEED / speed;
    vy_robot *= MAX_SPEED / speed;
    speed = MAX_SPEED;
  }

  move(st, atan2(vy_robot, vx_robot), speed, m1, m2, m3);
}
