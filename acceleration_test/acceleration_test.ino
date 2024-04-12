
//      ******************************************************************
//      *                                                                *
//      *      Example using units of Millimeters, instead of Steps      *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// Many stepper motors are used in projects where the motions are linear 
// instead of rotational.  Examples are XY mechanisms such as plotters and 
// 3D printers.  In these cases issuing move commands in "Millimeters" is  
// much more intuitive than "Steps".  This program show how to work with  
// units in millimeters.
//
// Note: If your mechanism has a Homing Switch so that it can automatically  
// move to its "Home" position on startup, checkout the function:  
//    moveToHomeInMillimeters()
//  
//
// Documentation at:
//    https://github.com/Stan-Reifel/SpeedyStepper
//
//
// The motor must be connected to the Arduino with a driver board having a 
// "Step and Direction" interface.  It's VERY important that you set the 
// motor current first!  Read the driver board's documentation to learn how.

// ***********************************************************************


#include <SpeedyStepper.h>
// pin assignments
const int MOTORLA_STEP_PIN = 9; // Orange
const int MOTORLA_DIRECTION_PIN = 8; // Yellow

const int MOTOR1_STEP_PIN = 5; // Orange
const int MOTOR1_DIRECTION_PIN = 4; // Yellow

const int MOTOR2_STEP_PIN = 7; // Purple
const int MOTOR2_DIRECTION_PIN = 6; // Yellow

SpeedyStepper stepperLA;
SpeedyStepper stepper1;
SpeedyStepper stepper2;

void homeLA (){
  const float homingSpeedInMMPerSec = 25.0*4;
  const float maxHomingDistanceInMM = 720*4;   // since my lead-screw is 38cm long, should never move more than that
  const int directionTowardHome = 1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward

  stepperLA.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, maxHomingDistanceInMM, 23);

  stepperLA.setStepsPerMillimeter(40 * 4);
  stepperLA.setSpeedInMillimetersPerSecond(26);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(50.0);
  stepperLA.moveToPositionInMillimeters(-10.0*10);
  //delay(5000);
}

void setup() 
{
  Serial.begin(9600);
  stepperLA.connectToPins(MOTORLA_STEP_PIN, MOTORLA_DIRECTION_PIN);
  stepper1.connectToPins(MOTOR1_STEP_PIN, MOTOR1_DIRECTION_PIN);
  stepper2.connectToPins(MOTOR2_STEP_PIN, MOTOR2_DIRECTION_PIN);
}



void loop() 
{
  //
  // First you must tell the library "how many steps it takes to move
  // one millimeter".  In this example I'm using this stepper motor with 
  // a built in lead-screw:
  //    Stepper Motor with 38cm Lead Screw:  https://www.pololu.com/product/2690
  //
  stepperLA.setStepsPerMillimeter(40 * 4);    // 4x microstepping
  homeLA();
  stepperLA.setSpeedInMillimetersPerSecond(40.0);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(10.0);

  // increase the speed, then go to position -16mm
  //
  while(true){
    
    stepperLA.moveToPositionInMillimeters(-20.0*10);
    delay(2000);
    
    stepperLA.moveToPositionInMillimeters(-10.0*10);
    delay(2000);
  }


}
