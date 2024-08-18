/* test servos on sockinator */
#include <Servo.h>

Servo arm;
Servo gripper;

void setup() {
  arm.attach(9);
  gripper.attach(10);// attaches the servo on pin 9 to the servo object
}

void loop() {
    arm.write(90);
    gripper.write(128); // needs to be between 128 (open) and 88 (closed)
    delay(5000);
    arm.write(80);
    gripper.write(88);
    delay(5000);
}
