#include "SnowtudoNav.h"
#include <Enes100.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Field layout (from the arena rules). Obstacles are 0.2 m (x) by 0.5 m (y).
// Two columns at x = 1.5 and 2.3, three lanes at y = 0.5 / 1.0 / 1.5.
// Exactly one obstacle per lane (row), three obstacles total.
// ---------------------------------------------------------------------------
namespace {
const int   kNumCols = 2;
const int   kNumLanes = 3;
const float kColX[kNumCols]   = {1.5f, 2.3f};        // obstacle column centres (m)
const float kLaneY[kNumLanes] = {0.5f, 1.0f, 1.5f};  // obstacle row centres (m)
const float kObsHalfDepthM = 0.10f;                  // obstacle half depth in x
const float kExitM  = 0.30f;                         // drive this far past a column centre
const float kMaxY   = 1.80f;                         // furthest the robot centre can go
const float kXTolM  = 0.06f;

unsigned long timeLeft(unsigned long start, unsigned long total) {
    unsigned long e = millis() - start;
    return e >= total ? 0 : total - e;
}
}

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

// Burst length shrinks with the remaining distance so we stop instead of overshooting
unsigned int Navigator::burstFor(float distM) {
    float f = constrain(distM / slowDistM, 0.0f, 1.0f);
    return driveBurstMinMs + (unsigned int)((driveBurstMs - driveBurstMinMs) * f);
}

// ---------------------------------------------------------------------------
// Turn in place
// ---------------------------------------------------------------------------
bool Navigator::turnToHeading(float targetRad, float tolRad, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (!updatePose()) {          // camera lost us: hold still and retry
            _drive.stop();
            delay(settleMs);
            continue;
        }
        float err = angleDiff(targetRad, _theta);
        if (fabs(err) < tolRad) {
            _drive.stop();
            return true;
        }
        // Bigger error -> longer burst (clamped)
        unsigned int burst = turnBurstMinMs +
            (unsigned int)((turnBurstMaxMs - turnBurstMinMs) * min(1.0f, fabs(err) / PI));
        _drive.move(err > 0 ? Direction::TurnLeft : Direction::TurnRight);
        delay(burst);
        _drive.stop();
        delay(settleMs);
    }
    _drive.stop();
    return false;
}

// ---------------------------------------------------------------------------
// driveToPoint: face the goal point, then drive at it in bursts. No sonar.
// Fixes vs. the old version:
//  - stops re-aiming when very close (tiny position error = huge bearing error)
//  - treats "target is now behind us" as arrived (overshoot) instead of spinning around
//  - bursts shrink near the target
//  - inner turn can't outlive the outer timeout
//  - removed the front sonar check (sonar points 45 deg right, would false-trigger)
// ---------------------------------------------------------------------------
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

        float desired = atan2(dy, dx);
        float err = angleDiff(desired, _theta);
        bool steer = dist > noSteerM;

        // Close in and the target is now behind us: we overshot, call it arrived
        if (!steer && fabs(err) > HALF_PI) {
            _drive.stop();
            return true;
        }

        if (steer && fabs(err) > replanAngle) {
            // Off course: rotate back toward the target
            unsigned long left = timeLeft(start, timeoutMs);
            turnToHeading(desired, headingTol, left < 5000UL ? left : 5000UL);
            continue;
        }

        _drive.move(Direction::Forward);
        delay(burstFor(dist));
        _drive.stop();
        delay(settleMs);
    }
    _drive.stop();
    return false;
}

// ---------------------------------------------------------------------------
// Strafe sideways while facing +x
// ---------------------------------------------------------------------------
bool Navigator::strafeToY(float targetY, float tolM, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (!updatePose()) { _drive.stop(); delay(settleMs); continue; }

        if (fabs(angleDiff(0.0f, _theta)) > replanAngle) {   // keep facing +x
            turnToHeading(0.0f, headingTol, 3000);
            continue;
        }

        float err = targetY - _y;
        if (fabs(err) < tolM) { _drive.stop(); return true; }

        // Facing +x, Left = +y. If your robot goes the wrong way, swap these two.
        _drive.move(err > 0 ? Direction::Left : Direction::Right);
        delay(burstFor(fabs(err)));
        _drive.stop();
        delay(settleMs);
    }
    _drive.stop();
    return false;
}

// ---------------------------------------------------------------------------
// Drive along +x holding a lane (re-strafes if it drifts, re-turns if it yaws)
// ---------------------------------------------------------------------------
bool Navigator::driveForwardTo(float targetX, float laneY, float tolM,
                               unsigned long timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (!updatePose()) { _drive.stop(); delay(settleMs); continue; }

        if (targetX - _x < tolM) { _drive.stop(); return true; }

        if (fabs(angleDiff(0.0f, _theta)) > replanAngle) {
            turnToHeading(0.0f, headingTol, 3000);
            continue;
        }
        if (fabs(laneY - _y) > laneTolM) {                   // drifted out of the lane
            strafeToY(laneY, laneTolM * 0.5f, 3000);
            continue;
        }

        _drive.move(Direction::Forward);
        delay(burstFor(targetX - _x));
        _drive.stop();
        delay(settleMs);
    }
    _drive.stop();
    return false;
}

// ---------------------------------------------------------------------------
// Sonar probe (majority of 3 pings).
// The sensor points 45 deg right, so while the robot faces +x it sees the column
// face dx to its right at a range of dx*sqrt(2). A reading closer than that (plus a
// margin) means an obstacle is there. A free lane lets the beam through the gap, so it
// reads much farther or gets no echo ("no echo" counts as clear).
// ---------------------------------------------------------------------------
bool Navigator::probeBlocked(float expectedRangeM) {
    float limitCm = expectedRangeM * 100.0f + probeMarginCm;
    int hits = 0;
    for (int i = 0; i < 3; i++) {
        float d = _sonar->getDistanceCm();
        Serial.print("probe cm: "); Serial.print(d);
        Serial.print("  limit: ");  Serial.println(limitCm);
        if (d > 0 && d < limitCm) hits++;
        delay(60);                                           // let the echo die out
    }
    return hits >= 2;
}

// ---------------------------------------------------------------------------
// driveWithSensor: cross both obstacle columns, then drive to the goal.
//
// For each column:
//  1. drive to the standoff line (probeDxM in front of the column face)
//  2. try lanes nearest-first. To probe lane L, park the robot at
//     y = laneY + sonarRight + dx so the 45-degree beam lands on that lane's face.
//  3. line up in the free lane's centre and drive straight through
//
// Rule shortcuts (exactly one obstacle per row):
//  - a lane BLOCKED in column 0 is guaranteed FREE in column 1 (tried first, no probe)
//  - a lane FREE in column 0 is guaranteed BLOCKED in column 1 (skipped)
// ---------------------------------------------------------------------------
bool Navigator::driveWithSensor(float goalX, float goalY, float tolM,
                                unsigned long timeoutMs) {
    unsigned long start = millis();
    if (!_sonar) return false;

    bool probed[kNumLanes]  = {false, false, false};   // column 0 results by lane
    bool blocked[kNumLanes] = {false, false, false};

    while (!updatePose()) {                            // wait for the camera
        _drive.stop();
        delay(settleMs);
        if (timeLeft(start, timeoutMs) == 0) return false;
    }
    if (!turnToHeading(0.0f, headingTol, 4000)) return false;

    for (int c = 0; c < kNumCols; c++) {
        const float faceX  = kColX[c] - kObsHalfDepthM;          // face the robot sees
        const float standX = faceX - probeDxM - sonarFwdM;       // robot centre x for probing
        const float exitX  = kColX[c] + kExitM;

        // 1. get to the probing line (current y)
        if (_x < standX - kXTolM) {
            if (!driveForwardTo(standX, _y, kXTolM, timeLeft(start, timeoutMs))) return false;
        }
        const float dx = faceX - (_x + sonarFwdM);               // actual sensor-to-face distance

        // 2. lanes nearest-first
        int order[kNumLanes] = {0, 1, 2};
        for (int i = 0; i < kNumLanes - 1; i++)
            for (int j = i + 1; j < kNumLanes; j++)
                if (fabs(kLaneY[order[j]] - _y) < fabs(kLaneY[order[i]] - _y)) {
                    int t = order[i]; order[i] = order[j]; order[j] = t;
                }

        int chosen = -1;
        // pass 0: lanes guaranteed free (blocked in col 0). pass 1: everything else.
        for (int pass = 0; pass < 2 && chosen < 0; pass++) {
            for (int k = 0; k < kNumLanes; k++) {
                int L = order[k];
                bool mustBeFree    = (c == 1 && probed[L] && blocked[L]);
                bool mustBeBlocked = (c == 1 && probed[L] && !blocked[L]);
                if (mustBeBlocked) continue;                     // one obstacle per row
                if ((pass == 0) != mustBeFree) continue;

                if (mustBeFree) { chosen = L; break; }           // no probe needed

                // Park so the 45-degree beam lands on the centre of lane L's face
                float probeY = kLaneY[L] + sonarRightM + dx;
                if (probeY > kMaxY) probeY = kMaxY;              // top lane: a few cm off-centre is fine
                if (!strafeToY(probeY, laneTolM, 6000)) return false;
                delay(settleMs);

                bool b = probeBlocked(dx * 1.4142f);
                Serial.print("col "); Serial.print(c);
                Serial.print(" lane "); Serial.print(L);
                Serial.println(b ? " BLOCKED" : " free");
                if (c == 0) { probed[L] = true; blocked[L] = b; }
                if (!b) { chosen = L; break; }
            }
        }
        if (chosen < 0) return false;                            // shouldn't happen with 3 obstacles

        // 3. line up in the lane centre and drive straight through
        if (!strafeToY(kLaneY[chosen], laneTolM, 6000)) return false;
        if (!driveForwardTo(exitX, kLaneY[chosen], kXTolM, timeLeft(start, timeoutMs))) return false;
    }

    // 4. open zone to the goal
    return driveToPoint(goalX, goalY, tolM, timeLeft(start, timeoutMs));
}

// ---------------------------------------------------------------------------
// Whole course. Goal is roughly (4, 1.8), but the marker can't reach the wall,
// so aim a bit short.
// ---------------------------------------------------------------------------
bool Navigator::CompleteCourse() {
    return driveWithSensor(3.7f, 1.8f);
}