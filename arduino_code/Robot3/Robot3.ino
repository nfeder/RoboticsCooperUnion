//Libraries
#include <HC_SR04.h>
#include <PID_v1.h>
#include <motor_control.h>
#include <Servo.h>

#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0

//Global variables
HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);
Servo actuator;

//Set up PID controller variables
double setpoint = 80;
double input;
double output;
double kp = 11;
double ki = 0;
double kd = 1;
PID controller(&input, &output, &setpoint, kp, ki, kd, REVERSE);

void setup() {
  // put your setup code here, to run once:
  motor_setup();

  //setup servo and point forward
  actuator.attach(9);
  actuator.write(90);

  //initialize distance sensor
  sensor.begin();
  sensor.start();

  //set PID parameters
  controller.SetOutputLimits(-255, 255);
  controller.SetSampleTime(25);
  controller.SetMode(1);

  //begin serial for debugging
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //get distance value from sensor, update input to be distance value
  if (sensor.isFinished()) {
    input = sensor.getRange();
    sensor.start(); //restart the sensor
  }

  //compute PID controller once input has been updated
  controller.Compute();

  //set motor power based on output from PID controller
  raw_motor_control(output*1.1425, output);
  Serial.print("output: ");
  Serial.print(output);
  Serial.print(" distance: ");
  Serial.println(input);
}
