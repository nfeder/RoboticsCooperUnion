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
  int max_index = max_distance();
  int min_index = min_distance();

  //update motion control parameters based on object detection output, adjust distance threshold here
  update_motion_control(max_index, min_index, thresh);

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
int max_distance()
{
  //Check each part of the distance array
  int max_index = find_max(mysodar.distance, mysodar.array_length);
  return max_index; //Return maximum index value
}
int min_distance()
{
  //Check each part of the distance array
  int min_index = find_min(mysodar.distance, mysodar.array_length);
  return min_index; //Return maximum index value
}

//Based on the output from object_detection, State accordingly
void update_motion_control(int max_index, int min_index, int threshold)
{
  //Get minimum value at min_index
  int max_val = mysodar.distance[max_index];
  int min_val = mysodar.distance[min_index];

  //If the min_val is < threshold, update state accordingly
  if (/*min_val*/ mysodar.distance[2] < threshold) {
    Serial.print("max_index: "); Serial.println(max_index);
    
    //nested if statements to decide turning etc...
    //Note that the parameters in mysodar made the array length 5 in this case
    //There is a way to generalize this for any array size but for simplicity
    //this is hardcoded for a 5 length array.
    if (max_index == 3 || max_index == 4) { //slow down Turn left
      control_state.setRotationState(70);
      control_state.setLinearState(100);
      Serial.println("TURN LEFT");
    }
    else if (max_index == 1 || max_index == 2){ //slowdown Turn Right
      control_state.setRotationState(-70);
      control_state.setLinearState(100);
      Serial.println("TURN RIGHT");
    }
  }


  //Else reset controller state and return
  else {
    control_state.setRotationState(-1); //no rotation
    control_state.setLinearState(160); //drive forward
    return;
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
