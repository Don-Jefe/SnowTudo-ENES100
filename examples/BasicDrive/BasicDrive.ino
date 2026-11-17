#include <Enes100.h>
#include <SnowtudoRobot.h>

MecanumDrive drive;     // default pins: LF 2/3, LR 4/5, RF 6/7, RR 8/9
Ultrasonic sonar;       // default pins: TRIG 11, ECHO 10

void setup() {
    // Check the signature against File > Examples > ENES100 for your library version
    Enes100.begin("Snowtudo", MATERIAL, 49, 1116, 12, 13);

    drive.begin();
    sonar.begin();
}

void loop() {
    if (sonar.obstacleInFront(20)) {
        drive.move(Direction::Stop);
    } else {
        drive.move(Direction::Forward);
    }
}
