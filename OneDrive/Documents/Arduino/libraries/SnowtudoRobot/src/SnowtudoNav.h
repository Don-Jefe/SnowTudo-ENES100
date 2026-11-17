#ifndef SNOWTUDO_NAV_H
#define SNOWTUDO_NAV_H

#include <Arduino.h>
#include "SnowtudoRobot.h"

// Navigation using the ENES100 Vision System (overhead camera).
// Coordinates are in meters (arena is 4 m x 2 m), heading in radians (-pi..pi,
// 0 = facing +x, positive = counter-clockwise).
//
// Motors are driven with PWM through the drivetrain's shared enable pin.
// Navigator runs continuously and scales power with the remaining error:
// full power when far away, ramping down to a minimum power near the target
// so it doesn't overshoot.

class Navigator {
public:
    Navigator(MecanumDrive &drive, Ultrasonic *sonar = nullptr);

    // Latest pose, updated by updatePose(). Returns false if the OTV isn't visible.
    bool updatePose();
    float x() const { return _x; }
    float y() const { return _y; }
    float theta() const { return _theta; }

    // Rotate in place until facing targetRad (within tolRad). Returns true on success.
    bool turnToHeading(float targetRad, float tolRad = 0.1f, unsigned long timeoutMs = 10000);

    // Face the point, then drive toward it until within tolM meters.
    // Returns true if it arrived, false on timeout or if the sonar sees an obstacle
    // closer than obstacleCm (check lastStopWasObstacle() to tell which).
    bool driveToPoint(float targetX, float targetY, float tolM = 0.08f,
                      unsigned long timeoutMs = 30000);

    bool lastStopWasObstacle() const { return _obstacle; }

    // Tuning (defaults are a starting point; adjust on the real robot)
    float obstacleCm   = 20.0f;   // stop if sonar reads closer than this
    float headingTol   = 0.10f;   // rad, how well aligned counts as "pointed at it"
    float replanAngle  = 0.25f;   // rad, re-turn if drift exceeds this while driving

    // PWM power, 0-255. The min values must be high enough to overcome stiction.
    uint8_t driveMaxPwm    = 220;
    uint8_t driveMinPwm    = 120;
    float   driveSlowDistM = 0.40f;  // m, start ramping power down inside this distance
    uint8_t turnMaxPwm     = 150;
    uint8_t turnMinPwm     = 50;
    float   turnSlowRad    = 1.80f;  // rad, start ramping power down inside this error

    unsigned int loopMs   = 20;   // control loop period while moving
    unsigned int settleMs = 120;  // wait used when the camera loses the OTV

    // Smallest signed angle difference a - b, wrapped to (-pi, pi]
    static float angleDiff(float a, float b);

private:
    // Linear map: frac 0 -> lo, frac 1 -> hi (frac clamped to 0..1)
    static uint8_t scalePwm(float frac, uint8_t lo, uint8_t hi);

    MecanumDrive &_drive;
    Ultrasonic *_sonar;
    float _x = 0, _y = 0, _theta = 0;
    bool _obstacle = false;
};

#endif