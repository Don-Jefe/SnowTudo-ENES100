// Value & constants
enum dirrections {
    Forward,
    Right,
    Backward,
    Left,
    TurnLeft,
    TurnRight,
    DiagRight,
    DiagLeft,
    Stop
};
float Position[2];





// Pin Setup
// LeftFront = LF
// LeftRear = LR
// RightFront = RF
// RightRear = RR
//Sensor and sensor functions


// Left H bridge
const int LF_IN1 = 2, LF_IN2 = 3;
const int LR_IN1 = 4, LR_IN2 = 5;

// Right H bridge
const int RF_IN1 = 6, RF_IN2 = 7;
const int RR_IN1 = 8, RR_IN2 = 9;

// Ultra Sonic Sensor
const int ECHO = 10, TRIG = 11;

// Force Sensor
//const int Force = Analoginput pin;

// Ultra Sonic Sensor
float GetDistanceToObstacle() {

}
bool ObstacleInFront() {};

// WiFi camera positions

float GetXPos() {};
float GetYPos() {};


// Drive train and drive functions
// Sets the direction and power of the Motor
// Inputs: input pins for motor in1 & in2 as well as a direction
//  1 = forward
// -1 = backward
//  0 or anything else == stop
void SetMotor(int in1, int in2, int dir) {
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

// Input the motor directions for all four motors
// Order is ==> Left Front ==> Left Rear ==> Right Front ==> Right Rear
void Drive(int lf, int lr, int rf, int rr) {
    SetMotor(LF_IN1, LF_IN2, lf);
    SetMotor(LR_IN1, LR_IN2, lr);
    SetMotor(RF_IN1, RF_IN2, rf);
    SetMotor(RR_IN1, RR_IN2, rr);
}

void DriveDirection(enum dirrections dir) {
    switch(dir){
        
        case Forward:
            Drive(1, 1, 1, 1);
            break;

        case Backward:
            Drive(-1, -1, -1, -1);
            break;
        
        case Right:
            Drive(1, -1, -1, 1);
            break;
        
        case Left:
            Drive(-1, 1, 1, -1);
            break;
        
        case TurnLeft:
            Drive(-1, -1, 1, 1);
            break;

        case TurnRight:
            Drive(1, 1, -1, -1);
            break;

        case DiagLeft:
            // idk Drive();
            break;
        
        case DiagRight:
            // idk
            break;
        case Stop:
            Drive(0, 0, 0, 0);
            break;
    }
}






void setup() {

}

void loop() {
    
}