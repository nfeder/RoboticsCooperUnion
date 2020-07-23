//Import libraries
#include <motor_control.h>
#include <HC_SR04.h>
#include <SODAR.h>
#include <State.h>
#include <PID_v1.h>
#include <MPU6050.h>

// Servo Pin declaration
#define SERVO_PIN 9

//Ultrasonic distance sensor pin declaration
#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0
#define CENTER 91
#define RESOLUTION 30
#define UPPER_RANGE 151
#define LOWER_RANGE 31

//Global Variables
//Distance threshold variable
int thresh = 50;

//SODAR
//NOTE: in SODAR.cpp, remember to adjust the estimated_servo_response_time variable accordingly (on line 22)
SODAR mysodar(TRIG_PIN, ECHO_PIN, ECHO_INT, SERVO_PIN, 91, RESOLUTION, UPPER_RANGE, LOWER_RANGE);
//You can also adjust it since it's a public variable (you'll have to do this in void setup()
//mysodar.estimated_servo_response_time = 100;

//State
State control_state;

//PID for driving straight
double input, output, setpoint;
double kp = 5;
double ki = 5;
double kd = 0;
PID controller(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//Accelerometer/Gyro
MPU6050 IMU(4, 5);

void setup() {
  //Initialize Motors
  motor_setup();

  //Make servo piin an output
  pinMode(SERVO_PIN, OUTPUT);

  //Initialize Sodar
  //adjust estaimted servo response time
  mysodar.estimated_servo_response_time = 100;
  mysodar.initialize_arrays();
  mysodar.init();

  //Initialize State
  control_state.setLinearState(160); //Set linear state to drive forward
  control_state.setRotationState(-1); //Set rotation state to drive straight

  //Initialize PID controller hyper params
  controller.SetOutputLimits(-95, 95);
  controller.SetSampleTime(25); //Set sample time to 25ms interval
  controller.SetMode(1); //Turns on the PID controller

  //Initialize MPU6050
  IMU.initialize();
  IMU.update();

  //begin serial for debugging
  Serial.begin(9600);
}

void loop() {
  //Update the sodar to move to next position etc...
  mysodar.update();

  //Update the IMU
  IMU.update();

  //object detection
  int min_index = min_distance();

  //update motion control parameters based on object detection output, adjust distance threshold here
  update_motion_control(min_index, thresh);

  //Update PID parameters
  setpoint = control_state.getRotationState();
  input = IMU.get_ang_vel('z');
  
  //Compute PID
  controller.Compute();

  //Update power to motors
  raw_motor_control(control_state.getLinearState() - output, control_state.getLinearState() + output);

  //Print distance array for debugging
  print_array(mysodar.distance, mysodar.array_length);
}

//algorithm to "find and object"
int min_distance()
{
  //Check each part of the distance array
  int min_index = find_min(mysodar.distance, mysodar.array_length);
  return min_index; //Return maximum index value
}

//Based on the output from object_detection, State accordingly
void update_motion_control(int min_index, int threshold)
{
  int i = 1;
  
  //Get minimum value at min_index
  int min_val = mysodar.distance[min_index];

  //If the straight ahead is < threshold, update state accordingly
  if (mysodar.distance[2] < threshold) {
    if (min_index == 3 || min_index == 4) { //slow down Turn left
      i = -1;
    }

    control_state.setRotationState(360 * i);
  }
  else {
    control_state.setRotationState(-1);
  }
}

//Pass in an array and it's length, return the index the maximum is at
int find_max(int dist[], int arr_size)
{
  float maximum = dist[0];
  int max_index = 0;
  //Start at index 1 so we don't have to have complex logic inside the for loop
  for (int i = 1; i < arr_size; i++)
  {
    if (dist[i] > dist[max_index] && dist[i] != 1213) {
      max_index = i; //if new value is less than minimum, set min_index to new value index
      maximum = dist[i];
    }
  }
  return max_index;
}

//Pass in an array and it's length, return the index the minimum is at
int find_min(int dist[], int arr_size)
{
  float minimum = dist[0];
  int min_index = 0;
  //Start at index 1 so we don't have to have complex logic inside the for loop
  for (int i = 1; i < arr_size; i++)
  {
    if (dist[i] < dist[min_index]) {
      min_index = i; //if new value is less than minimum, set min_index to new value index
      minimum = dist[i];
    }
  }
  return min_index;
}


void print_array(int arr[], int len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.print(arr[i]);
    Serial.print(" | ");
  }
  Serial.println();
}
//Thank you to "jazzycamel" from instructables on this great 'non-blocking' library for the ultrasonic distance sensor
// https://www.instructables.com/id/Non-blocking-Ultrasonic-Sensor-for-Arduino/
//Library name HC_SR04.h/.cpp
//#include <State.h>
//#include <PID_v1.h>
//#include <MPU6050.h>
//#include <Servo.h>
//#include <motor_control.h>
//#include <HC_SR04.h>
//
////State
//State control_state;
//
////PID for driving straight
//double input, output, setpoint;
//double kp = 5;
//double ki = 5;
//double kd = 0;
//PID controller(&input, &output, &setpoint, kp, ki, kd, DIRECT);
//
////Accelerometer/Gyro
//MPU6050 IMU(4, 5);
//
//
//// Servo Pin declaration
//#define SERVO_PIN 9  //servo connect to D9
//
////Ultrasonic distance sensor pin declaration
//#define TRIG_PIN 10
//#define ECHO_PIN 2
//#define ECHO_INT 0
//
////Resolution and limits of searching parameters (for  ultrasonic distance sensor)
//#define RESOLUTION 10 //How many degrees to have between detections (this value should be an integer divisior of MAXDEG
//#define MAXDEG 130 //Angle of maximum detection
//#define MINDEG 50 //Angle of minimum detection
//
////Global Variables
////Initialize orientations servo will point to
//float orientation[((MAXDEG - MINDEG) / RESOLUTION) + 1];
//const int array_length = (sizeof(orientation) / sizeof(orientation[0]));
//float dist[array_length];
//int current_index = 0; // This global variable will be used to iterate through the orientation array to point the sensor in the right direction
//
//
//Servo head; //Servo Variable
//HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT); //Non-blocking ultrasonic distance sensor
//
////------------------------------------------------------------
//
////------------------------------------------------------------
////SETUP AND LOOP FUNCTIONS
//void setup() {
//  pinMode(TRIG_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);
//  digitalWrite(TRIG_PIN, LOW); //Initialize trigger pin for ultrasonic sensor to LOW
//  
//  /*init servo*/
//  head.attach(SERVO_PIN);
//  head.write(90);
//  delay(1000);
//
//  //Initialize serial communication
//  Serial.begin(9600);
//
//  //Start the ultrasonic distance sensor
//  sensor.begin(); // Make sure to run the begin function, this will setup the interrupt pins to make the sensor functional
//  sensor.start(); 
//
//  //Initialize State
//  control_state.setLinearState(160); //Set linear state to drive forward
//  control_state.setRotationState(-1); //Set rotation state to drive straight
//
//  //Initialize PID controller hyper params
//  controller.SetOutputLimits(-95, 95);
//  controller.SetSampleTime(25); //Set sample time to 25ms interval
//  controller.SetMode(1); //Turns on the PID controller
//
//  //Initialize MPU6050
//  IMU.initialize();
//  IMU.update();
//
//  //Initialize orientations array
//  for (int i = 0; i < array_length; i++)
//  {
//    orientation[i] = (RESOLUTION * i) + MINDEG;
//  }
//  Serial.print(orientation[0]);
//
//}
//
//
//void loop() {
//  //Update PID parameters
//  IMU.update();
//  setpoint = control_state.getRotationState();
//  input = IMU.get_ang_vel('z');
//  
//  //Compute PID
//  controller.Compute();
//
//  //Check if sensor has finished it's last detection run, if so update motor control
//  if(sensor.isFinished())
//  {
//    //Update motor control
//    int control_value = discrete_detection(orientation, dist, array_length, 50); //Get control command from detection
//    setpoint = control_state.getRotationState();
//    raw_motor_control(control_state.getLinearState() - output, control_state.getLinearState() + output);
//    
//    //Run detect next, which will rotate servo and start next detection loop
//    detect_next();
//  }
//}
//void detect_next()
//{
//  //Update distance array with value from sensor (we know it's finished because this function is only called when sensor.isFinished() is true
//  dist[current_index] = sensor.getRange();
//  
//  //Use current index to point ultrasonic distance sensor
//  head.write(orientation[current_index]);
//  
//  //Give the sevo a few milliseconds to get to position
//  delay(200);
//
//  //Iterate index, if it's too high re-set to 0 index
//  current_index++;
//  if(current_index == array_length) { current_index = 0; }
//
//  //Restart detection
//  sensor.start();
//}
//
//
////OTHER FUNCTIONS-----------------------------------------------------------------------------------
//
//int analyze_data()
//{
//  //Analyze dist[] array, and determine where center of object is
//
//  //Simple analysis, pick the minimum and return the angle to point towards
//  return find_min(dist, sizeof(dist) / sizeof(dist[0]));
//
//}
//
//int discrete_detection(float o[], float dist[], int len, int thresh)
//{
//    // Check if we're clear of objects
//    int control = check_distance(o, dist, len, thresh);
//    return control;
//
//}
//
////Check to make sure that all distances in range are > thresh
//int check_distance(float o[], float dist[], int len, int thresh)
//{
//    //Separate array indexs into left, center, right
//    int center_index = int(len/2); //casting to int will truncate remainder if value is odd
//    int center_window = 1; //How many indicies around center_index will we count as "center"
//
//    //Get the minimum distance value and index
//    int min_index = find_min(dist, len);
//    int min_value = dist[min_index];
//
//    //Check if minimum above threshold, if so return 0 to move foward
//    if(min_value > thresh) { return 0; }
//
//    //If minimum is within threshold, determine if it exists in right, center, or left, return appropriate 
//    //control direction
//    //Right segment
//    if(min_index < center_index-center_window){ return 3; } //Turn left
//
//    //Center segment
//    if(min_index < center_index+center_window && min_index > center_index-center_window)
//    {
//        return 1; //Move backwards
//    }
//
//    //Left segment
//    if(min_index < len && min_index > center_index+center_window) { return 2; }//Turn right
//
//}
//
//int find_min(float dist[], int arr_size)
//{
//  float minimum = dist[0];
//  int min_index = 0;
//  //Start at index 1 so we don't have to have complex logic inside the for loop
//  for (int i = 1; i < arr_size; i++)
//  {
//    if (dist[i] < dist[min_index]) {
//      min_index = i; //if new value is less than minimum, set min_index to new value index
//      minimum = dist[i];
//    }
//  }
//  return min_index;
//}
//
//void update_motion_control(int index, int threshold)
//{
//  int i = 1;
//
//  //If the straight ahead is < threshold, update state accordingly
//  if (index != 0) {
//    if (index == 2) { //slow down Turn left
//      i = -1;
//    }
//
//    control_state.setRotationState(360 * i);
//  }
//  else {
//    control_state.setRotationState(-1);
//  }
//}
