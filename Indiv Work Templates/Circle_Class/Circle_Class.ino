C// Circle Class
class Circle {
   // Private variables that can only be accessed within the class
   // Place private variables here.
   int radius;
   
  // Everything after "public" can be accesed outside of the class (i.e in "setup" or "loop")
  public:
    // Put a constructor here: What information is needed?
    Circle(int);
    // Create a function that returns area (use 3.14 for pi)
    double area() {
      return 3.14 * radius * radius;
    }
    // Create a function that allows the user to update the values
    void setRadius(int);
};

// Definitions of functions declared above
Circle::Circle(int r) {
  radius = r;
}
void Circle::setRadius(int r) {
  radius = r;
}

void setup() {
  // Instantiates the Circle class
  Circle circ(2);

  // Can only print after serial.begin()
  Serial.begin(9600);
  
  // Calls the area() function within the class
 double a = circ.area();

  // Prints area
  Serial.println(a);

  // Updates values
  circ.setRadius(4);

  // Prints new areas
  a = circ.area();
  Serial.println(a);
  
}

void loop() {
  // put your main code here, to run repeatedly:
}
