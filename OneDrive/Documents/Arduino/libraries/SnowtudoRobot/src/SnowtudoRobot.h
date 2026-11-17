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
//
// All motor drivers share ONE enable pin (default 11, PWM-capable on the UNO).
// Direction comes from each motor's IN pins; speed is a single PWM value (0-255)
// applied to every wheel through the shared enable pin.
class MecanumDrive {
public:
    MecanumDrive(int lfIn1 = 8, int lfIn2 = 9,
                 int lrIn1 = 6, int lrIn2 = 7,
                 int rfIn1 = 2, int rfIn2 = 3,
                 int rrIn1 = 4, int rrIn2 = 5,
                 int enablePin = 11);

    // Call once in setup(): sets pin modes and stops the motors
    void begin();

    // Shared PWM power for all wheels, 0-255. Applied on the next drive()/move().
    void setSpeed(uint8_t pwm);
    uint8_t speed() const { return _speed; }

    // dir: 1 = forward, -1 = backward, anything else = stop
    void setMotor(int in1, int in2, int dir);

    // Directions per motor, order: LF, LR, RF, RR
    void drive(int lf, int lr, int rf, int rr);

    // Complex movement
    void move(Direction dir);
    void stop();

private:
    int _pins[8];
    int _en;
    uint8_t _speed = 255;
};

// HC-SR04 style ultrasonic sensor
class Ultrasonic {
public:
    Ultrasonic(int trigPin = A2, int echoPin = A3);

    void begin();

    // Distance in cm, or -1 if no echo (out of range / timeout)
    float getDistanceCm();

    // True if something is closer than thresholdCm (valid readings only)
    bool obstacleInFront(float thresholdCm = 20.0);

private:
    int _trig, _echo;
};

class ForceSensor {
public:
    ForceSensor(int pin = A0);

    void begin();

    // Returns a value between 0 and 1023
    int readRaw();

    // Returns a value between 0.0 and 1.0
    float readNormalized();

    // Output 1 for light, 2 for medium, 3 for heavy
    int getWeight();

private:
    int _pin;
};


#endif