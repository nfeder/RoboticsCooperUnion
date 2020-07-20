//Import libraries
#include <motor_control.h>

#include <MPU6050.h>
#define SDA 4
#define SCL 5
MPU6050 sensor(SDA, SCL);

#include <PID_v1.h>

#include <State.h>

//Set up State for driving straight controller
State controller_state_straight;

//Set up PID controller variables for driving straight
double setpoint_straight = -1;
double input_straight;
double output_straight;
double kps = 5;
double kis = 5;
double kds = 0;
PID controller_s(&input_straight, &output_straight, &setpoint_straight, kps, kis, kds, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //initialize motor
  motor_setup();

  //initialize MPU6050
  sensor.initialize();
  sensor.update();

  //set PID parameters`
  controller_s.SetOutputLimits(-95, 95);
  controller_s.SetSampleTime(25);
  controller_s.SetMode(1);

  //initialize State
  controller_state_straight.setLinearState(160);
  controller_state_straight.setRotationState(-1);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //get input (and print it for debugging)
  sensor.update();
  double angVelReading = sensor.get_ang_vel('z');
  Serial.print("angular velocity for z: ");
  Serial.print(angVelReading);
  input_straight = angVelReading;

  //compute PID controllers after input is updated
  controller_s.Compute();
//  controller_d.Compute();

  //set motor power based on output from PID controllers
  raw_motor_control(160 - output_straight, 160 + output_straight);

}
