//global variables
int frontDistance;
int rightDistance;
int leftDistance;
int limit = 1000;
int startTurn = -1;

//set up distance sensor and servo
#include <Servo.h>
#include <HC_SR04.h>

#define SERVO_PIN 9
#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0
#define CENTER 90
#define DESIRED_DISTANCE 50

Servo actuator;
HC_SR04 dist_sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);

//set up motor
#include <motor_control.h>

//set up MPU6050
#include <MPU6050.h>
#define SDA 4
#define SCL 5
MPU6050 sensor(SDA, SCL);

//set up PID
#include <PID_v1.h>
double setpoint = -1;
double input;
double output;
double kp = 5;
double ki = 5;
double kd = 0;
PID controller(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//set up State for the PID
#include <State.h>
State controller_state;

void setup() {
  // put your setup code here, to run once:
  //initialize serial monitor for debugging
   Serial.begin(9600);
  
  //initialize motor setup
    motor_setup();

   //initialize MPU6050
    sensor.initialize();
    sensor.update();

  //initialize State variables
   controller_state.setLinearState(160);
   controller_state.setRotationState(-1);

  //set PID parameters
    controller.SetOutputLimits(-95, 95);
    controller.SetSampleTime(25);
    controller.SetMode(1);

  //initialize distance sensor
  dist_sensor.begin();
  dist_sensor.start();

  //Attach servo to servo pin, and rotate to point sensor forward
  actuator.attach(SERVO_PIN);
  actuator.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  // get angular velocity input for PID 
  sensor.update();
  double angVelReading = sensor.get_ang_vel('z');
  input = angVelReading;

  // compute PID
  controller.Compute();

  // set direction based on State
  setpoint = controller_state.getRotationState();
  
  // set motor power based on PID
  raw_motor_control(controller_state.getLinearState() - output , controller_state.getLinearState() + output);

  // check front distance with sensor
  actuator.write(90);
  delay(1000);
  if(dist_sensor.isFinished()) {
    frontDistance = getDistance();
  }

  Serial.println(frontDistance);

  //keep changing the limit every second
  if (millis() > limit) {
    limit += 1000;
  }

  //only do this when the servo is close-ish to a wall in the front
  if (frontDistance < 100) {
  // move servo and check left distance once a second
//  if (millis() > limit) {
   actuator.write(45);
   if(dist_sensor.isFinished()) {
    
     leftDistance = getDistance();
   }
   delay(500);

   // compare front and left distances and move accordingly
   if (leftDistance > frontDistance) {
    controller_state.setRotationState(-45);
    startTurn = millis();
   }

   // else compare right distance and front distance and move accordingly
   else {
     actuator.write(103);
     if(dist_sensor.isFinished()) {
       rightDistance = getDistance();
     }
     delay(500);

     if (rightDistance > frontDistance) {
       controller_state.setRotationState(45);
       startTurn = millis();
     }
     else {
      controller_state.setRotationState(0);
     }
   }

   if ((startTurn == -1) || (millis() - startTurn == 100)) {
    controller_state.setRotationState(0);
   }
  }
//  }
}

//create a method to check distance
int getDistance() {
  int distance = dist_sensor.getRange();
  dist_sensor.start();
  return distance;
}
