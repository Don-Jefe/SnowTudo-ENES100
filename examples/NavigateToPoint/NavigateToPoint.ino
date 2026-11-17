#include <Enes100.h>
#include <SnowtudoRobot.h>
#include <SnowtudoNav.h>

MecanumDrive drive;
Ultrasonic sonar;
Navigator nav(drive, &sonar);

void setup() {
    // Confirm this signature against File > Examples > ENES100
    Enes100.begin("Snowtudo", MATERIAL, 49, 1116, 12, 13);
    drive.begin();
    sonar.begin();
    Enes100.println("Snowtudo ready");
}

void loop() {
    // Example: go to (2.0 m, 1.0 m), then face +x
    if (nav.driveToPoint(2.0, 1.0)) {
        nav.turnToHeading(0);
        Enes100.println("Arrived");
        while (true) { }   // done
    } else if (nav.lastStopWasObstacle()) {
        Enes100.println("Obstacle! need to go around");
        // TODO: obstacle avoidance (strafe Left/Right, then call driveToPoint again)
        drive.move(Direction::Left);
        delay(400);
        drive.stop();
    }
}
