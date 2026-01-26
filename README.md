# HCR-OmniwheelBase

We're using a [12V ST3215 Smart Motor](https://www.dfrobot.com/product-2962.html), which we can communicate with over UART.

Should be able to use Arduino libraries like [LX16AServo](https://github.com/madhephaestus/lx16a-servo).

The code below can be used to control robot velocity.

```
    // =======================
    // Robot parameters
    // =======================
    const float MAX_WHEEL_SPEED = 100.0;  // arbitrary units (RPM, % speed, etc.)

    // Wheel angles in radians
    const float W1_ANGLE = PI / 2;        // 90°
    const float W2_ANGLE = 7 * PI / 6;    // 210°
    const float W3_ANGLE = 11 * PI / 6;   // 330°

    // =======================
    // Placeholder servo API
    // =======================
    void setWheelSpeed(int wheelID, float speed) {
    // wheelID: 1, 2, 3
    // speed: signed value
    // Replace this with your servo library call
    }

    // =======================
    // Omni kinematics
    // =======================
    void driveOmni(float vx, float vy, float omega) {
    float w1 = -sin(W1_ANGLE) * vx + cos(W1_ANGLE) * vy + omega;
    float w2 = -sin(W2_ANGLE) * vx + cos(W2_ANGLE) * vy + omega;
    float w3 = -sin(W3_ANGLE) * vx + cos(W3_ANGLE) * vy + omega;

    // Normalize speeds
    float maxMag = max(max(abs(w1), abs(w2)), abs(w3));
    if (maxMag > 1.0) {
        w1 /= maxMag;
        w2 /= maxMag;
        w3 /= maxMag;
    }

    // Scale to usable speed
    setWheelSpeed(1, w1 * MAX_WHEEL_SPEED);
    setWheelSpeed(2, w2 * MAX_WHEEL_SPEED);
    setWheelSpeed(3, w3 * MAX_WHEEL_SPEED);
    }
```

Alternatively, we can use the position control of the motors and the size of the wheels to control the robot's position directly (i.e: move 1 metre forward) using code like this:

```
    #include <Arduino.h>
    #include <math.h>

    const float WHEEL_RADIUS = 0.05; // meters
    const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;

    // Wheel geometry in radians
    const float W1_ANGLE = PI / 2;       // 90°
    const float W2_ANGLE = 7 * PI / 6;   // 210°
    const float W3_ANGLE = 11 * PI / 6;  // 330°

    const int SERVO_ID_1 = 1;
    const int SERVO_ID_2 = 2;
    const int SERVO_ID_3 = 3;

    // Placeholder function — replace with your library
    void rotateServoRelative(int servoID, float degrees) {
        // Example:
        // myServoLibrary.setRelativePosition(servoID, degrees);
        Serial.printf("Servo %d rotate %.2f degrees\n", servoID, degrees);
    }

    // =======================
    // Move robot forward/backward
    // distance_m: positive = forward, negative = backward
    // =======================
    void moveForward(float distance_m) {
        // Calculate the rotation in degrees for a wheel to move that distance
        float rotationDegrees = (distance_m / WHEEL_CIRCUMFERENCE) * 360.0;

        // 3-wheel omni kinematics
        float w1 = -sin(W1_ANGLE) * 0 + cos(W1_ANGLE) * distance_m + 0; // vx=0, omega=0
        float w2 = -sin(W2_ANGLE) * 0 + cos(W2_ANGLE) * distance_m + 0;
        float w3 = -sin(W3_ANGLE) * 0 + cos(W3_ANGLE) * distance_m + 0;

        // Normalize relative rotations
        float maxMag = max(max(abs(w1), abs(w2)), abs(w3));
        if (maxMag > 1.0) {
            w1 /= maxMag;
            w2 /= maxMag;
            w3 /= maxMag;
        }

        // Scale by rotationDegrees
        w1 *= rotationDegrees;
        w2 *= rotationDegrees;
        w3 *= rotationDegrees;

        // Send commands to servos
        rotateServoRelative(SERVO_ID_1, w1);
        rotateServoRelative(SERVO_ID_2, w2);
        rotateServoRelative(SERVO_ID_3, w3);
    }

    // =======================
    // Example usage
    // =======================
    void setup() {
        Serial.begin(115200);
        delay(1000);

        Serial.println("Moving robot forward 30cm...");
        moveForward(0.30); // move 0.3 meters forward
    }

    void loop() {
        // nothing here
    }
```
