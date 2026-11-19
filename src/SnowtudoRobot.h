#ifndef SNOWTUDO_ROBOT_H
#define SNOWTUDO_ROBOT_H

#include <Arduino.h>

// Movement directions for the mecanum drive train
enum class Direction {
    Forward,
    Backward,
    Right,      // strafe right
    Left,       // strafe left
    TurnLeft,   // rotate in place
    TurnRight,
    DiagRight,  // forward + strafe right
    DiagLeft,   // forward + strafe left
    Stop
};

// Naming: LF = left front, LR = left rear, RF = right front, RR = right rear
class MecanumDrive {
public:
    MecanumDrive(int lfIn1 = 8, int lfIn2 = 9,
                 int lrIn1 = 6, int lrIn2 = 7,
                 int rfIn1 = 2, int rfIn2 = 3,
                 int rrIn1 = 4, int rrIn2 = 5);

    // Call once in setup(): sets pin modes and stops the motors
    void begin();

    // dir: 1 = forward, -1 = backward, anything else = stop
    void setMotor(int in1, int in2, int dir);

    // Directions per motor, order: LF, LR, RF, RR
    void drive(int lf, int lr, int rf, int rr);

    // Complex movement
    void move(Direction dir);
    void stop();

private:
    int _pins[8];
};

// HC-SR04 style ultrasonic sensor
class Ultrasonic {
public:
    Ultrasonic(int trigPin = 11, int echoPin = 10);

    void begin();

    // Distance in cm, or -1 if no echo (out of range / timeout)
    float getDistanceCm();

    // True if something is closer than thresholdCm (valid readings only)
    bool obstacleInFront(float thresholdCm = 20.0);

private:
    int _trig, _echo;
};

// ---------------------------------------------------------------------------
// ForceSensor is disabled for now (it had compile errors). Re-enable later.
// ---------------------------------------------------------------------------
/*
class ForceSensor {
public:
    ForceSensor(int pin = A0) : _pin(pin) {}

    void begin();

    // Returns a value between 0 and 1023
    int readRaw();

    // Returns a value between 0.0 and 1.0
    float readNormalized();

    // 1 = light, 2 = medium, 3 = heavy
    int getWeight();

private:
    int _pin;
};
*/

#endif