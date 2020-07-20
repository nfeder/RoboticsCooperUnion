//Import libraries
#include <motor_control.h>

#include <MPU6050.h>
#define SDA 4
#define SCL 5
MPU6050 sensor(SDA, SCL);

//Global variables
int leftMotor;
int rightMotor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  motor_setup();

  sensor.initialize();
  sensor.update();

  //Create starter values for left and right motor powers
  leftMotor = 150 * 1.1425;
  rightMotor = 150;
}

void loop() {
  // put your main code here, to run repeatedly:
  raw_motor_control(leftMotor, rightMotor);

  sensor.update();
  double angVelReading = sensor.get_ang_vel('z');
  Serial.print("angular velocity for z: ");
  Serial.println(angVelReading);

  if (angVelReading > 5) {
    leftMotor += 5 * 1.1425;
  }
  if (angVelReading < -5) {
    rightMotor += 5;
  }
}
