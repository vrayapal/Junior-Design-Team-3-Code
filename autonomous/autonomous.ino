#include <Servo.h>
#include <ezButton.h>
#include <SpeedyStepper.h>
#include "SparkFun_OPT4048.h"
#include <Wire.h>

SparkFun_OPT4048 myColor;

ezButton limitSwitch1(23); // Limit 1
ezButton limitSwitch2(25); // Limit 2
ezButton limitSwitch3(27); // Limit 3

ezButton limitSwitchL(29); // Button 1 Left  // G-> Digital; Y-> 5V; S-> GND
ezButton limitSwitchR(33); // Button 2 Right // G-> Digital; Y-> 5V; S-> GND

//Limit Switch Variables
bool limit1 = false;
bool limit2 = false;
bool limit3 = false;

int height = 0;
int b=0;
int g=0;
int r=0;

// Button Switch Varaiables
bool LeftButton = false;
bool RightButton = false;

// Gripper Variables
Servo gripper_servo;
int gripper_pos = 180;        // variable for servo position
bool gripper_state = false; // Initializes in the open state
int gripper_closed = 105;     // servo value at which the servo is closed 
int speed = 8; // smaller is faster
int gripper_open = 180;      // servo value at which the servo is closed
int color = 0;              // Initialises variable to hold the color value fromt eh color sensor

// Joystick 1 Variables
int x1Pin = A1;      // variable for joystick pin  (digital)
int y1Pin = A0;      // variable for joystick pin  (digital)
int buttonPin1 = 2;  // variable for button pin (analog in) SW

double xPosition1 = 0;      //position in the x for the joystick
double yPosition1 = 0;      //position in the y for the joystick
int buttonState1 = 0;    //position of the button for the joystick

// Joystick 2 Variables
int x2Pin = A3;      // variable for joystick pin  (digital)
int y2Pin = A2;      // variable for joystick pin  (digital)
int buttonPin2 = 3;  // variable for button pin (analog in) SW

double xPosition2 = 0;      //position in the x for the joystick
double yPosition2 = 0;      //position in the y for the joystick
int buttonState2 = 0;    //position of the button for the joystick

//Stepper Pins
const int MOTORLA_STEP_PIN = 9; // Orange
const int MOTORLA_DIRECTION_PIN = 8; // Yellow

const int MOTOR1_STEP_PIN = 5; // Orange
const int MOTOR1_DIRECTION_PIN = 4; // Yellow

const int MOTOR2_STEP_PIN = 7; // Purple
const int MOTOR2_DIRECTION_PIN = 6; // Yellow
const int ENA_PIN = 38; // Brown
// ENA pin can be used to diable stepper motor to allow free movement

bool grip = true; // true is closed // false is closed 

SpeedyStepper stepperLA;
SpeedyStepper stepper1;
SpeedyStepper stepper2;

float startlocations[9][3] = {  //    {"RA1 position","RA2 Position","zlocation index"}
  {-0.85, -1.10, -24},     // Tower 1a
  {-0.85, -1.10, -24},    // Tower 1b
  {-0.85, -1.10, -24},    // Tower 1c
  {-0.85, -1.10, -24},    // Tower 2a
  {-0.85, -1.10, -24},   // Tower 2b
  {-0.85, -1.10, -24},   // Tower 2c
  {-0.85, -1.10, -24},   // Tower 3a
  {-0.85, -1.10, -24},   // Tower 3b
  {-0.85, -1.10, -24}   // Tower 3c
};

float caselocations[9][3] = {   //    {"RA1 position","RA2 Position","zlocation index"}
  {-0.55, -0.07, -389},  // Blue 1
  {-0.63, -0.50, -389},  // Blue 2
  {-1.04, -0.95, -389},  // Blue 3
  {-0.55, -0.07, -260},  // Red 4
  {-0.63, -0.50, -260},  // Red 5
  {-1.04, -0.95, -260},  // Red 6
  {-0.55, -0.07, -135},  // Green 7
  {-0.63, -0.50, -135},  // Green 8
  {-1.04, -0.95, -135}   // Green 9
};

int blue = 0;
int red = 3;
int green = 6;

float safelocation[3][2] = {
  {-0.74, -0.95},
  {-0.44, -0.59},
  {-0.21, -0.14}
  };    //    {"RA1 position","RA2 Position"} at the safe position

int zlocations[4] = {-24, -135, -260, -389}; // {tower position, }

void joystick() {
  xPosition1 = (analogRead(x1Pin)-511);   // normalized x position from -1 to 1 for joystick 1
  yPosition1 = (analogRead(y1Pin)-511);   // normalized y position from -1 to 1 for joystick 1
  buttonState1 = digitalRead(buttonPin1);

  xPosition2 = (analogRead(x2Pin)-511);   // normalized x position from -1 to 1 for joystick 2
  yPosition2 = (analogRead(y2Pin)-511);   // normalized y position from -1 to 1 for joystick 2
  buttonState2 = digitalRead(buttonPin2);
  
  // Debug Stuff
  if(buttonState1 == 0){
    Serial.print("X: ");
    Serial.print(xPosition1);
    Serial.print(" | Y: ");
    Serial.print(yPosition1);
    Serial.print(" | ButtonR: ");
    Serial.println(buttonState1);
  }
  if(buttonState2 == 0){
    Serial.print("X: ");
    Serial.print(xPosition2);
    Serial.print(" | Y: ");
    Serial.print(yPosition2);
    Serial.print(" | ButtonL: ");
    Serial.println(buttonState2);
  }
}

void buttons() {
  limitSwitchL.loop();
  int L = limitSwitchL.getState();
  if(L == LOW)
    LeftButton = false;
    Serial.println("Left Button Triggered");
  if (L == HIGH)
     LeftButton = true;
  
  limitSwitchR.loop();
  int R = limitSwitchR.getState();
  if(R == LOW)
    RightButton = false;
    Serial.println("Right Button Triggered");
  if (R == HIGH)
     RightButton = true;
}

void limit(){               // limit switch function for limit switches. To add more add another limit do the following: add a block in this function; intialize 'limit4' above; set debounce time in setup

  limitSwitch1.loop();
  limit1 = limitSwitch1.getState();
    if(limit1 == true){
      limit1 = true;
      Serial.println("limit1 has been triggered");
      //stepperLA.moveToPositionInMillimeters(-5.0*10);
    }
    else if(limit1 == false){
      limit1 = false;
      //Serial.println(limit1);

      //delay(100);
    }  
  limitSwitch2.loop();
  limit2 = limitSwitch2.getState();
    if(limit2 == true ){
      limit2 = true;
      Serial.println("limit2 has been triggered");
      //code to scoot robot away from limit switch 
      //stepper1.moveRelativeInSteps(-20*4);
    }
    else if(limit2 == false){
      limit2 = false;
    }

  limitSwitch3.loop();
  limit3 = limitSwitch3.getState();
    if(limit3 == true ){
      limit3 = true;
      Serial.println("limit2 has been triggered");
      //code scoot robot away from limit switch 
      //stepper2.moveRelativeInSteps(-20*4);
    }
    else if(limit3 == false){
      limit3 = false;
    }
}

void gripper(){   
  //delay(2000);                    // function to actuate gripper from open to closed and vice versa. Requires values to be plugged in from 
  //gripper_state = !gripper_state;
       // False is open; True is closed
  if (gripper_state == true){          // If gripper is going to close
  //Actuates the gripper from open to close position
    for (gripper_pos = gripper_closed; gripper_pos <= gripper_open; gripper_pos += 1) {  //change pos += 1 depending on if we are opening or closing servo
      gripper_servo.write(gripper_pos);
      //Serial.println(gripper_pos);
      delay(speed);                      // increase if servo moves too fast and vice versa
      //check grip
      //find color of block
    }
    grip = true;
  }
  else if (gripper_state == false){    //If gripper is going to open
  //Actuates the gripper from closed to open position
    for (gripper_pos = gripper_open; gripper_pos >= gripper_closed; gripper_pos -= 1) { //change pos -= 1 depending on if we are opening or closing servo
      gripper_servo.write(gripper_pos);
      //Serial.println(gripper_pos);
      delay(speed);                      // increase if servo moves too fast and vice versa
    }
    grip = false;
  }
}


void checkcolor(){
  if ((myColor.getCIEy()>= 0.20) && (myColor.getCIEy()<= 0.35) && (myColor.getCIEx()>= 0.35) && (myColor.getCIEx()<= 0.50)){
    color=0;
    Serial.println("Block is Blue");
  }
  else if ((myColor.getCIEx()>= 0.55) && (myColor.getCIEx()<= 0.70) && (myColor.getCIEy()>= 0.25) && (myColor.getCIEy()<= 0.35)){
    color=1;
    Serial.println("Block is Red");
  }
  else if ((myColor.getCIEx()>= 0.35) && (myColor.getCIEx()<= 0.55) && (myColor.getCIEy()>= 0.36) && (myColor.getCIEy()<= 0.50)){
    color=2;
    Serial.println("Block is Green");
  }
  else {
    Serial.println("Failed color check");
    checkcolor();

  }
}


void homeLA (){
  const float homingSpeedInMMPerSec = 25.0*4;
  const float maxHomingDistanceInMM = 720*4;   // since my lead-screw is 38cm long, should never move more than that
  const int directionTowardHome = 1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward

  stepperLA.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, maxHomingDistanceInMM, 23);

  stepperLA.setStepsPerMillimeter(40 * 4);
  stepperLA.setSpeedInMillimetersPerSecond(26);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(50.0);
  stepperLA.moveToPositionInMillimeters(-2.0*10);
  //delay(5000);
}

void home1 (){
  float homingSpeedInStepsPerSec = 50.0;
  float maxHomingDistanceInMM = 1440;   // since my lead-screw is 38cm long, should never move more than that
  int directionTowardHome = 1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward
  
  digitalWrite(ENA_PIN, HIGH); // disable stepper 2 while homing stepper 1
  
  Serial.println("Homing Stepper 1");
  stepper1.moveToHomeInSteps(directionTowardHome, homingSpeedInStepsPerSec, maxHomingDistanceInMM, 25);
  Serial.println("Homed Stepper 1");
  
  digitalWrite(ENA_PIN, LOW); // renable stepper 2 after homing stepper 1

  homingSpeedInStepsPerSec = 50.0*2;
  maxHomingDistanceInMM = 1440;
  directionTowardHome = 1;
  Serial.println("Homing Stepper 2");
  stepper2.moveToHomeInSteps(directionTowardHome, homingSpeedInStepsPerSec, maxHomingDistanceInMM, 27);
  Serial.println("Homed Stepper 2");
  
  stepper1.setSpeedInStepsPerSecond(400*4);
  stepper1.setAccelerationInStepsPerSecondPerSecond(800*2);
  stepper1.moveRelativeInSteps(-20*4);
  Serial.println("RA1 Start Position");
  

  stepper2.setSpeedInStepsPerSecond(400*4);
  stepper2.setAccelerationInStepsPerSecondPerSecond(800*2);
  stepper2.moveRelativeInSteps(-20*4);
  Serial.println("RA2 Start Position");
  //delay(1000);
}

void safe() {
  stepper1.moveToPositionInRevolutions(safelocation[height][0]);
  stepper2.moveToPositionInRevolutions(safelocation[height][1]);
  Serial.print("Moved to safe Position: ");
  Serial.println(height);
}

void setup() {
  Serial.begin(9600);
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  limitSwitch3.setDebounceTime(50);

  pinMode(x1Pin, INPUT);
  pinMode(y1Pin, INPUT);
  pinMode(buttonPin1, INPUT_PULLUP); 
  
  pinMode(x2Pin, INPUT);
  pinMode(y2Pin, INPUT);
  pinMode(buttonPin2, INPUT_PULLUP); 

  limitSwitchL.setDebounceTime(50);
  limitSwitchR.setDebounceTime(50);
  
  gripper_servo.attach(12); //PWM
  // Brown -> Gnd // Orange -> PWM // Red -> Vcc

  stepperLA.connectToPins(MOTORLA_STEP_PIN, MOTORLA_DIRECTION_PIN);
  stepper1.connectToPins(MOTOR1_STEP_PIN, MOTOR1_DIRECTION_PIN);
  stepper2.connectToPins(MOTOR2_STEP_PIN, MOTOR2_DIRECTION_PIN);
  pinMode(ENA_PIN, OUTPUT);

  Wire.begin();
    if (!myColor.begin()) {
        Serial.println("OPT4048 not detected- check wiring or that your I2C address is correct!");
        while (1) ;
    }
  myColor.setBasicSetup();
}

void loop() {
  //Serial.println("Hello World");
  limit();
  //Serial.println("Hello World1");
  joystick();
  //Serial.println("Hello World2");
  buttons();
  //Serial.println("Hello World3");
  Serial.println("Homing P1");
  homeLA();
  Serial.println("Homing RA 1 and 2");
  home1();
  Serial.println("Homing RA 1 and 2");

  stepperLA.setStepsPerMillimeter(40 * 4); // times 4x microstepping
  stepperLA.setSpeedInMillimetersPerSecond(22);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(50.0);
  
  stepper1.setStepsPerRevolution(200*4); // times 8x microstepping
  stepper1.setSpeedInRevolutionsPerSecond(1.0);
  stepper1.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  stepper2.setStepsPerRevolution(200*8); // times 8x microstepping
  stepper2.setSpeedInRevolutionsPerSecond(1.0);
  stepper2.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
  
  gripper_state=true;
  gripper();
  
  while(true){  
    buttons();
    if (LeftButton == false){
      for (int i = 0; i <= 8; i++) {
        // Open Gripper
        gripper_state=true;
        gripper();
        //Move to pick up location
        stepperLA.moveToPositionInMillimeters(zlocations[0]);
        stepper1.moveToPositionInRevolutions(startlocations[i][0]/2.0);
        stepper2.moveToPositionInRevolutions(startlocations[i][1]/2.0);
        stepper1.moveToPositionInRevolutions(startlocations[i][0]);
        stepper2.moveToPositionInRevolutions(startlocations[i][1]);
        Serial.println("Pick up location");
        //wait
        delay(2500);
        //close gripper
        gripper_state=false;
        gripper();
        delay(2500);
        //find color
        //checkcolor();
        
        if (color = 0){           // Blue
          Serial.println("Grabbed block is blue");
          height = b;
          stepper2.moveToPositionInRevolutions(safelocation[b][1]/2.0);
          stepper1.moveToPositionInRevolutions(caselocations[b][0]/2.0);
          stepper2.moveToPositionInRevolutions(safelocation[b][1]);
          stepper1.moveToPositionInRevolutions(caselocations[b][0]);
          
          stepper2.moveToPositionInRevolutions(caselocations[blue][1]/2.0);
          stepper1.moveToPositionInRevolutions(caselocations[blue][0]/2.0);
          stepper2.moveToPositionInRevolutions(caselocations[blue][1]);
          stepper1.moveToPositionInRevolutions(caselocations[blue][0]);
          stepperLA.moveToPositionInMillimeters(caselocations[blue][2]);
          safe();
          blue++;
          b++;
          
        }
        else if (color = 1){      // Red
          Serial.println("Grabbed block is blue");
          height = r;
          safe();
          stepperLA.moveToPositionInMillimeters(caselocations[red][2]/2.0);
          stepper2.moveToPositionInRevolutions(caselocations[red][1]/2.0);
          stepper1.moveToPositionInRevolutions(caselocations[red][0]/2.0);
          stepperLA.moveToPositionInMillimeters(caselocations[red][2]);
          stepper2.moveToPositionInRevolutions(caselocations[red][1]);
          stepper1.moveToPositionInRevolutions(caselocations[red][0]);
          gripper_state=false;
          gripper();
          safe();
          red++;
          r++;
        }
        else if(color = 2){       // Green
          Serial.println("Grabbed block is blue");
          height = g;
          safe();
          stepperLA.moveToPositionInMillimeters(caselocations[green][2]/2.0);
          stepper1.moveToPositionInRevolutions(caselocations[green][0]/2.0);
          stepper2.moveToPositionInRevolutions(caselocations[green][1]/2.0);
          stepperLA.moveToPositionInMillimeters(caselocations[green][2]);
          stepper1.moveToPositionInRevolutions(caselocations[green][0]);
          stepper2.moveToPositionInRevolutions(caselocations[green][1]);
          gripper_state=false;
          gripper();
          safe();
          green++;
          g++;
        }


      }
    }
  }
}


