#include <Servo.h>
Servo servo1;

#include <motor_control.h>

#include <MPU6050.h>
#define SDA 4
#define SCL 5
MPU6050 sensor(SDA, SCL);

void setup() {
  // put your setup code here, to run once:
  servo1.attach(9);

  motor_setup();
  
  Serial.begin(9600);

  sensor.initialize();
  sensor.update();
}

void loop() {
  // put your main code here, to run repeatedly:
  forward(200);
  delay(500);
diff_right(200);
  delay(500);
  
  sensor.update();
  double accelReading = sensor.get_accel('z');
  Serial.print("accel for z: ");
  Serial.println(accelReading);

//  double angle = accelReading * 90 - 90;
  int angle = 90;
  servo1.write(angle);
}
