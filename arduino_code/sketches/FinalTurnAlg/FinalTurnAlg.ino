//global variables
int distance;

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
  // put your main code here, to run repeatedly
  // get angular velocity input for PID 
  sensor.update();
  double angVelReading = sensor.get_ang_vel('z');
  input = angVelReading;

  // compute PID
  controller.Compute();

  // check distance with sensor
  if(dist_sensor.isFinished()) {
    distance = dist_sensor.getRange();
    dist_sensor.start();
  }


  // if distance is low, slow down and change direction using State
  if (distance < DESIRED_DISTANCE) {
    controller_state.setRotationState(360);
  }

  // if distance is high, make direction forward and speed up (so it will
  //reset when distance is high again)
  else {
    controller_state.setRotationState(-1);
  }

  // set direction based on State
  setpoint = controller_state.getRotationState();
  
  // set motor power based on PID
  raw_motor_control(controller_state.getLinearState() - output , controller_state.getLinearState() + output);
}
