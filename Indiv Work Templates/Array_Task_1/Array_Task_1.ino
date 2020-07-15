// Array Task 1

void setup() {
  Serial.begin(9600);
  int ar1[10];
  int i = 0;
//  
  while (i < 11) {
    ar1[i] = i * 3;
    i++;
  }

  for (int i = 0; i < 11; i++) {
    ar1[i] = i * 3;
  }

  for (int i = 0; i < 11; i++) {
    Serial.print(ar1[i]);
    Serial.print(" ");
  }
  
  Serial.println();

  for (int i = 0; i < 11; i++) {
    if (ar1[i] % 9 == 0) {
      Serial.println(ar1[i]);
    }
  }

  // Creat an array of type int and size 10:

  // Using a while loop, fill the array with multiples of 3 between 3 and 30 (inclusive):
                // Remember the syntax?
                // while (condition) {
                //       code...
                // }



  // Rewrite the code above with a for loop:
                // Remember the syntax?
                // for (initialization; condition; update variable) {
                //       code...
                // }



  // Write a loop that will go through the array 
  //     and print any values that are a multiple of 9:
                // Hint: Use the % (modulus) operator.


}

void loop() {
  // put your main code here, to run repeatedly:

}
