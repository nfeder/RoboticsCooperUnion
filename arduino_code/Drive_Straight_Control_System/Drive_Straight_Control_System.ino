//Import libraries
#include <motor_control.h>

#include <MPU6050.h>
#define SDA 4
#define SCL 5
MPU6050 sensor(SDA, SCL);

#include <PID_v1.h>

#include <State.h>

//Set up State
State controller_state;

//Set up PID controller variables
double setpoint = 0;
double input;
double output;
double kp = 5;
double ki = 5;
double kd = 0;
PID controller(&input, &output, &setpoint, kp, ki, kd, DIRECT);

int limit = 2000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //initialize motor
  motor_setup();

  //initialize MPU6050
  sensor.initialize();
  sensor.update();

  //set PID parameters`
  controller.SetOutputLimits(-95, 95);
  controller.SetSampleTime(25);
  controller.SetMode(1);

  //initialize State
  controller_state.setLinearState(160);
  controller_state.setRotationState(45);
}

void loop() {
  // put your main code here, to run repeatedly:
  //get the setpoint from the State object
  setpoint = controller_state.getRotationState();
  
  //get input (and print it for debugging)
  sensor.update();
  double angVelReading = sensor.get_ang_vel('z');
//  Serial.print("angular velocity for z: ");
//  Serial.print(angVelReading);
  input = angVelReading;

  //compute PID controller after input is updated
  controller.Compute();

  //set motor power based on output from PID controller and print
  //for debugging
  raw_motor_control(controller_state.getLinearState() - output , controller_state.getLinearState() + output);
//  Serial.print(" output: ");
//  Serial.println(output);

  //make the robot change directions every 2 seconds
  if (millis() > limit) {
    controller_state.setLinearState(controller_state.getLinearState() * -1);
    controller_state.setRotationState(controller_state.getRotationState() * -1);
    limit += 2000;
  }
//
  Serial.println(millis());
}
