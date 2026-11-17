#include "SnowtudoNav.h"
#include <Enes100.h>
#include <math.h>

Navigator::Navigator(MecanumDrive &drive, Ultrasonic *sonar)
    : _drive(drive), _sonar(sonar) {}

bool Navigator::updatePose() {
    if (!Enes100.isVisible()) return false;
    _x = Enes100.getX();
    _y = Enes100.getY();
    _theta = Enes100.getTheta();
    return true;
}

float Navigator::angleDiff(float a, float b) {
    float d = a - b;
    while (d > PI)   d -= 2 * PI;
    while (d <= -PI) d += 2 * PI;
    return d;
}

uint8_t Navigator::scalePwm(float frac, uint8_t lo, uint8_t hi) {
    frac = constrain(frac, 0.0f, 1.0f);
    return lo + (uint8_t)((hi - lo) * frac);
}

bool Navigator::turnToHeading(float targetRad, float tolRad, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (!updatePose()) {          // camera lost
            Serial.print("Help Im lost! ");
            _drive.stop();
            delay(settleMs);
            continue;
        }
        float err = angleDiff(targetRad, _theta);
        if (fabs(err) < tolRad) {
            _drive.stop();
            return true;
        }
        // Bigger error -> more power, ramping down to turnMinPwm as we close in
        _drive.setSpeed(scalePwm(fabs(err) / turnSlowRad, turnMinPwm, turnMaxPwm));
        _drive.move(err > 0 ? Direction::TurnLeft : Direction::TurnRight);
        delay(loopMs);
    }
    _drive.stop();
    return false;
}

bool Navigator::driveToPoint(float targetX, float targetY, float tolM,
                             unsigned long timeoutMs) {
    unsigned long start = millis();
    _obstacle = false;

    while (millis() - start < timeoutMs) {
        if (!updatePose()) {
            _drive.stop();
            delay(settleMs);
            continue;
        }

        float dx = targetX - _x;
        float dy = targetY - _y;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < tolM) {
            _drive.stop();
            return true;
        }

        // if (_sonar && _sonar->obstacleInFront(obstacleCm)) {
        //     _drive.stop();
        //     _obstacle = true;
        //     return false;
        // }

        float desired = atan2(dy, dx);
        float err = angleDiff(desired, _theta);

        if (fabs(err) > replanAngle) {
            // Off course: stop and rotate back toward the target
            _drive.stop();
            turnToHeading(desired, headingTol, 5000);
        } else {
            // Full power when far, ramping down to driveMinPwm near the target
            _drive.setSpeed(scalePwm(dist / driveSlowDistM, driveMinPwm, driveMaxPwm));
            _drive.move(Direction::Forward);
            delay(loopMs);
        }
    }
    _drive.stop();
    return false;
}