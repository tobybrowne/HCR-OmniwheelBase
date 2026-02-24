#pragma once

#define USE_EKF   0   // set to 1 to re-enable EKF
#define DEMO_MODE 0   // set to 1 for circle demo (no WiFi input needed)

#define DEMO_RADIUS        0.5f   // metres
#define DEMO_ANGULAR_SPEED 1.5708f // rad/s  (PI/2)

#define S_RXD 16
#define S_TXD 17

#define OPERATION_TIME 44
#define OPERATION_MODE 33
#define TORQUE_SWITCH  40

#define WHEEL_RADIUS  0.03
#define ROBOT_RADIUS  0.250171
#define TICKS_PER_REV 4096

#define FIDUCIAL_INTERVAL_MS 2000
