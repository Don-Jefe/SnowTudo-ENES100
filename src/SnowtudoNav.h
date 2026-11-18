#ifndef SNOWTUDO_NAV_H
#define SNOWTUDO_NAV_H

#include <Arduino.h>
#include "SnowtudoRobot.h"

// Navigation using the ENES100 Vision System (overhead camera).
// Coordinates are in meters (arena is 4 m x 2 m), heading in radians (-pi..pi,
// 0 = facing +x, positive = counter-clockwise).
//
// The motors are on/off only (no PWM), so Navigator moves in short bursts:
// move a little -> stop -> let the camera update -> re-check -> repeat.
// That avoids overshooting.
//
// The ultrasonic sensor is mounted on the front of the robot and points 45 degrees
// to the RIGHT, so it can't see straight ahead. driveWithSensor() uses it to probe
// obstacle lanes from the side instead (see SnowtudoNav.cpp).

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

    // Face the point, then drive toward it until within tolM meters. No sonar.
    // Returns true if it arrived, false on timeout.
    bool driveToPoint(float targetX, float targetY, float tolM = 0.08f,
                      unsigned long timeoutMs = 30000);

    // Drive along +x (heading 0) to targetX while holding the lane at laneY
    bool driveForwardTo(float targetX, float laneY, float tolM = 0.06f,
                        unsigned long timeoutMs = 8000);

    // Slide sideways to targetY while facing +x
    bool strafeToY(float targetY, float tolM = 0.04f, unsigned long timeoutMs = 6000);

    // Get through the two obstacle columns using the sonar, then drive to the goal.
    bool driveWithSensor(float goalX, float goalY, float tolM = 0.10f,
                         unsigned long timeoutMs = 90000);

    // Whole mission course (edit as needed)
    bool CompleteCourse();

    bool lastStopWasObstacle() const { return _obstacle; }

    // ---------------- Tuning (adjust on the real robot) ----------------
    float obstacleCm   = 20.0f;   // (unused now: sonar points 45 deg right, not forward)
    float headingTol   = 0.10f;   // rad, how well aligned counts as "pointed at it"
    float replanAngle  = 0.25f;   // rad, re-turn if drift exceeds this while driving
    unsigned int driveBurstMs = 150;     // longest forward/strafe burst
    unsigned int driveBurstMinMs = 40;   // shortest burst, used right next to the target
    unsigned int turnBurstMinMs = 30;
    unsigned int turnBurstMaxMs = 120;
    unsigned int settleMs = 100;  // pause after each burst so vision catches up
    float slowDistM    = 0.30f;   // bursts shrink as the remaining distance drops below this
    float noSteerM     = 0.12f;   // inside this, stop re-aiming at the target
    float laneTolM     = 0.05f;   // lane drift allowed while driving through a column

    // Sonar mount in the robot frame (meters). Sensor aimed 45 deg to the right.
    float sonarFwdM     = 0.15f;  // how far ahead of the robot centre the sensor sits
    float sonarRightM   = 0.0f;   // how far right of the robot centre the sensor sits
    float probeDxM      = 0.25f;  // sensor-to-column-face distance while probing
    float probeMarginCm = 12.0f;  // lane is "blocked" if reading < dx*sqrt(2) + this

    // Smallest signed angle difference a - b, wrapped to (-pi, pi]
    static float angleDiff(float a, float b);

private:
    unsigned int burstFor(float distM);
    bool probeBlocked(float expectedRangeM);

    MecanumDrive &_drive;
    Ultrasonic *_sonar;
    float _x = 0, _y = 0, _theta = 0;
    bool _obstacle = false;
};

#endif