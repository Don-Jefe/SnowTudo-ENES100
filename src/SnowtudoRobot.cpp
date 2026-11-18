#include "SnowtudoRobot.h"

// ---------------- MecanumDrive ----------------

MecanumDrive::MecanumDrive(int lfIn1, int lfIn2, int lrIn1, int lrIn2,
                           int rfIn1, int rfIn2, int rrIn1, int rrIn2) {
    _pins[0] = lfIn1; _pins[1] = lfIn2;
    _pins[2] = lrIn1; _pins[3] = lrIn2;
    _pins[4] = rfIn1; _pins[5] = rfIn2;
    _pins[6] = rrIn1; _pins[7] = rrIn2;
}

void MecanumDrive::begin() {
    for (int i = 0; i < 8; i++) {
        pinMode(_pins[i], OUTPUT);
    }
    stop();
}

void MecanumDrive::setMotor(int in1, int in2, int dir) {
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

void MecanumDrive::drive(int lf, int lr, int rf, int rr) {
    setMotor(_pins[0], _pins[1], lf);
    setMotor(_pins[2], _pins[3], lr);
    setMotor(_pins[4], _pins[5], rf);
    setMotor(_pins[6], _pins[7], rr);
}

void MecanumDrive::move(Direction dir) {
    switch (dir) {
        case Direction::Forward:   drive( 1,  1,  1,  1); break;
        case Direction::Backward:  drive(-1, -1, -1, -1); break;
        case Direction::Right:     drive( 1, -1, -1,  1); break;
        case Direction::Left:      drive(-1,  1,  1, -1); break;
        case Direction::TurnLeft:  drive(-1, -1,  1,  1); break;
        case Direction::TurnRight: drive( 1,  1, -1, -1); break;
        // Diagonals: only one pair of wheels spins (LF+RR for right, LR+RF for left)
        case Direction::DiagRight: drive( 1,  0,  0,  1); break;
        case Direction::DiagLeft:  drive( 0,  1,  1,  0); break;
        case Direction::Stop:
        default:                   drive( 0,  0,  0,  0); break;
    }
}

void MecanumDrive::stop() {
    drive(0, 0, 0, 0);
}

// ---------------- Ultrasonic ----------------

Ultrasonic::Ultrasonic(int trigPin, int echoPin) : _trig(trigPin), _echo(echoPin) {}

void Ultrasonic::begin() {
    pinMode(_trig, OUTPUT);
    pinMode(_echo, INPUT);
    digitalWrite(_trig, LOW);
}

float Ultrasonic::getDistanceCm() {
    digitalWrite(_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);

    unsigned long duration = pulseIn(_echo, HIGH, 30000UL); // 30 ms timeout (~5 m)
    if (duration == 0) return -1;
    return duration * 0.0343f / 2.0f;
}

bool Ultrasonic::obstacleInFront(float thresholdCm) {
    float d = getDistanceCm();
    return (d > 0 && d < thresholdCm);
}

// ---------------------------------------------------------------------------
// ForceSensor is disabled for now. Re-enable together with the class in the .h
// ---------------------------------------------------------------------------
/*
void ForceSensor::begin() {
    pinMode(_pin, INPUT);
}

int ForceSensor::readRaw() {
    return analogRead(_pin);
}

float ForceSensor::readNormalized() {
    return readRaw() / 1023.0f;
}

// output 1 for light, output 2 for medium, output 3 for heavy
int ForceSensor::getWeight() {
    int r = readRaw();
    if (r < 30) return 1;
    else if (r < 40) return 2;
    else return 3;
}
*/