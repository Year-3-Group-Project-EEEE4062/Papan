// Driver module with inner and outer loop controller for autonomous boat.


#include <math.h>
#include "CytronMotorDriver.h"
#include "Boat.h"


// Configure the motor driver.
CytronMD motorLeft(PWM_DIR, 3, 4);   // PWM = Pin 3, DIR = Pin 4.
CytronMD motorRight(PWM_DIR, 5, 6);  // PWM = Pin 5, DIR = Pin 6.


double normSpeed[2];


// Function for setup.
void setup() {
  Serial.begin(9600);
}


// Function for loop.
void loop() {
  innerController(5/5.0, 0/180.0, 0, normSpeed);
  
  motorLeft.setSpeed(normSpeed[0]);
  motorRight.setSpeed(normSpeed[1]);

  Serial.print("Left = ");
  Serial.print(normSpeed[0]);
  Serial.print(", Right = ");
  Serial.println(normSpeed[1]);
}
