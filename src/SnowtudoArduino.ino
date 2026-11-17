#include <SnowtudoRobot.h>
#include <Enes100.h>
#include <SnowtudoNav.h>

MecanumDrive drive;
Ultrasonic sonar;
Navigator nav(drive, &sonar);
ForceSensor force;

// Arena positions
float Xpos = 0;
float Ypos = 0;
float initialTurn;
float missionX = 0;
float missionY = 0;
const float lowerMissionX = 0.425;
const float lowerMissionY = 0.377;
const float upperMissionX = 0.425;
const float upperMissionY = 1.452;
const float centerStartX = 1;
const float centerStartY = 1;
const float endX = 3.527;
const float endY = 1.452;
boolean isTop = true;




   

void setup() {
    Serial.begin(9600);
    Enes100.begin("Snowtudo", MATERIAL, 49, 1116);
    drive.begin();
    force.begin();
    sonar.begin();
    drive.stop();
    nav.updatePose();

    // Find where bot is starting
    if (nav.y() < 1) {
        missionX = lowerMissionX;
        missionY = lowerMissionY;
        initialTurn = -1*PI / 2;
    } else {
        missionX = upperMissionX;
        missionY = upperMissionY;
        initialTurn = 1*PI / 2;
    }   


    // Turn Towards Mission site
    if (nav.updatePose()) {
            Enes100.print("Starting heading: ");
            Enes100.println(nav.theta());
        } else {
            Enes100.println("Robot not visible to camera yet");
        }

        bool turned = nav.turnToHeading(initialTurn, 0.03f, 25000);
        drive.stop();
        Enes100.println(turned ? "Turn complete" : "Turn failed or timed out");

        

    if (nav.updatePose()) {
        Enes100.print("Starting block approach: ");
        bool drove = nav.driveToPoint(missionX,missionY,.03f,30000);
        Enes100.println(turned ? "Drive Complete" : "Drive failed or timed out");
    }
     drive.move(Direction::Forward);
     delay(5000);
     drive.move(Direction::Stop);
     drive.move(Direction::Backward);
     delay(1000);
     drive.move(Direction::Stop);



    if (nav.updatePose()) {
    Enes100.print("Starting block approach: ");
    nav.driveToPoint(1,1,.03f,30000);
    }
}






void loop() {
    //drive.move(Direction::Forward);
    drive.move(Direction::Forward);
    Enes100.print("Normalized Force: ");
    Enes100.println(force.readNormalized());
    Serial.print("Raw Force: ");
    Enes100.println(force.readRaw());
    
    delay(500);
}
