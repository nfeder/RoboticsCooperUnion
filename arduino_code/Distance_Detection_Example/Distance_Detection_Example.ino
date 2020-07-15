// This will demonstrate the HC_SR04 & Servo
// The car will check the distance of objects in front of it.
// If an object is within DESIRED_DISTANCE, then the car will stop.

#include <Servo.h>
#include <HC_SR04.h>
#include <motor_control.h>

#define SERVO_PIN 9
#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0
#define CENTER 90
#define DESIRED_DISTANCE 30

Servo actuator;
HC_SR04 dist_sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);

// Current distance to closest object
int distance; 

//Current angle for servo and which way to move
int angle = 90;
boolean countUp = true;

void setup() {
  Serial.begin(9600);

  //Attach servo to servo pin, and rotate to point sensor forward
  actuator.attach(SERVO_PIN);
  actuator.write(90);

  //Initialize distance sensor and start pinging for distances
  dist_sensor.begin();
  dist_sensor.start();

  //Set up motor
  motor_setup();
}

void loop() {
  //Moves servo, then changes servo angle for next time
  actuator.write(angle);
  if (countUp && angle < 105) {
    angle++;
  }
  else if (countUp) {
    countUp = false;
    angle --;
    delay(500);
  }
  else if (!countUp && angle > 75) {
    angle--;
  }
  else {
    countUp = true;
    angle ++;
    delay(500);
  }
  
  // Gets distance
  if(dist_sensor.isFinished()) {
    distance = dist_sensor.getRange();
    dist_sensor.start();
  }

//  Serial.println(distance);

  // Stops motor if within DESIRED_DISTANCE, otherwise drives
  if(distance <= DESIRED_DISTANCE){
    raw_motor_control(0,0);
  } else {
    forward(200);
  }
  
  //Doing poorly: won't stop if it hits at a slight angle
}
