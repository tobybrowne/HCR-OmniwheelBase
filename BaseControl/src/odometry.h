#pragma once

#include <Arduino.h>
#include <math.h>
#include <SCServo.h>
#include "config.h"
#include "EKF.h"

extern float x, y, theta;
extern long prevEnc1, prevEnc2, prevEnc3;
extern EKF3 ekf;
extern unsigned long lastFiducialTime;
extern int32_t marker_x, marker_y, marker_theta;

void updatePos(SMS_STS st) {
  int enc1 = st.ReadPos(1);
  int enc2 = st.ReadPos(2);
  int enc3 = st.ReadPos(3);

  int d1 = enc1 - prevEnc1;
  int d2 = enc2 - prevEnc2;
  int d3 = enc3 - prevEnc3;

  if(d1 > TICKS_PER_REV/2)       d1 -= TICKS_PER_REV;
  else if(d1 < -TICKS_PER_REV/2) d1 += TICKS_PER_REV;
  if(d2 > TICKS_PER_REV/2)       d2 -= TICKS_PER_REV;
  else if(d2 < -TICKS_PER_REV/2) d2 += TICKS_PER_REV;
  if(d3 > TICKS_PER_REV/2)       d3 -= TICKS_PER_REV;
  else if(d3 < -TICKS_PER_REV/2) d3 += TICKS_PER_REV;

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
  double xdir     = a1x + a3x;
  double ydir     = -(a1y + a2y + a3y);

#if USE_EKF
  ekf.predictOmni((float)xdir, (float)ydir, (float)thetadir);
  x     = ekf.x[0];
  y     = ekf.x[1];
  theta = ekf.x[2];
#else
  x     += (float)xdir;
  y     += (float)ydir;
  theta += (float)thetadir;
#endif
}

void calibratePos() {
  x = 0; y = 0; theta = 0;
  ekf.x[0] = 0; ekf.x[1] = 0; ekf.x[2] = 0;
}

void checkFiducial() {
  unsigned long now = millis();
  if (now - lastFiducialTime < FIDUCIAL_INTERVAL_MS) return;
  lastFiducialTime = now;

  float measX     = (float)marker_x     / 1000.0f;
  float measY     = (float)marker_y     / 1000.0f;
  float measTheta = (float)marker_theta / 1000.0f;

  bool accepted = ekf.updateFiducial(measX, measY, measTheta);
  if (accepted) {
    x     = ekf.x[0];
    y     = ekf.x[1];
    theta = ekf.x[2];
    // Serial.print("[Fiducial] meas=( ");
    // Serial.print(measX, 3); Serial.print(", ");
    // Serial.print(measY, 3); Serial.print(")  EKF=( ");
    // Serial.print(x, 4);     Serial.print(", ");
    // Serial.print(y, 4);     Serial.println(")");
  }
}
