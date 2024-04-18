#include <Servo.h>
#include <ezButton.h>
#include <SpeedyStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

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

int check =0;

// Button Switch Varaiables
bool LeftButton = false;
bool RightButton = false;

// Gripper Variables
Servo gripper_servo;
int gripper_pos = 110;        // variable for servo position
bool gripper_state = false; // Initializes in the open state
int gripper_closed = 45;     // servo value at which the servo is closed 
int speed = 8; // smaller is faster
int gripper_open = 175;      // servo value at which the servo is closed
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

int zlocations[5] = {-5.63, -181.88, -307.51, -435.63, -70.01}; // {tower position, }


float startlocations[9][3] = {  //    {"RA1 position","RA2 Position","zlocation index"}
  {-0.46, -0.56, -5.63},     // Tower 1a
  {-0.55, -0.70, -5.63},    // Tower 1b
  {-0.70, -0.88, -5.63},    // Tower 1c
  {-0.63, -0.56, -5.63},    // Tower 2a
  {-0.74, -0.71, -5.63},   // Tower 2b
  {-0.81, -0.79, -5.63},   // Tower 2c
  {-0.81, -0.48, -5.63},   // Tower 3a
  {-0.87, -0.57, -5.63},   // Tower 3b
  {-0.94, -0.67, -5.63}   // Tower 3c
};

float caselocations[9][3] = {   //    {"RA1 position","RA2 Position","zlocation index"}
  {-0.66, 0.03, zlocations[3]},  // Blue 1
  {-0.68, -0.49, zlocations[3]},  // Blue 2
  {-1.08, -0.89, zlocations[3]},  // Blue 3
  {-0.66, 0.03, zlocations[2]},  // Red 4
  {-0.68, -0.49, zlocations[2]},  // Red 5
  {-1.08, -0.89, zlocations[2]},  // Red 6
  {-0.66, 0.03, zlocations[1]},  // Green 7
  {-0.68, -0.49, zlocations[1]},  // Green 8
  {-1.08, -0.89, zlocations[1]}   // Green 9
};

int blue = 0;
int red = 3;
int green = 6;

float safelocation[3][2] = {
  {-0.32, 0.03},
  {-0.42, -0.49},
  {-0.87, -1.05}
  };    //    {"RA1 position","RA2 Position"} at the safe position

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

double hue, saturation, lighting, value;

float fract(float x) { return x - int(x); }

float mix(float a, float b, float t) { return a + (b - a) * t; }

float step(float e, float x) { return x < e ? 0.0 : 1.0; }

float hsv[3];

float* rgb2hsv(float r, float g, float b, float* hsv) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  hsv[0] = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  hsv[1] = d / (qx + 1e-10);
  hsv[2] = qx;
  return hsv;
}

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
  if(L == LOW){
    LeftButton = false;
  }
  if (L == HIGH){
    LeftButton = true;
    //Serial.println("Left Button Triggered");
  }
  
  limitSwitchR.loop();
  int R = limitSwitchR.getState();
  if(R == LOW){
    RightButton = false;
  }
    
  if (R == HIGH){
    RightButton = true;
    //Serial.println("Right Button Triggered");
  }
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
      Serial.println("limit3 has been triggered");
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
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
 
  //Serial.print("R:\t"); Serial.print(int(red));
  //Serial.print("\tG:\t"); Serial.print(int(green));
  //Serial.print("\tB:\t"); Serial.print(int(blue));

  rgb2hsv(red, green, blue, hsv);
  /*
  Serial.print("H:\t");     Serial.print(hsv[0]);
  Serial.print("\tS:\t");   Serial.print(hsv[1]);
  Serial.print("\tV:\t");   Serial.println(hsv[2]);
  */

  if ((hsv[0]>= 0.0) && (hsv[0]<= 0.1) && (hsv[1]>= 0.36) && (hsv[1]<= 0.80) && (hsv[2]>= 100.0) && (hsv[2]<= 190.0)){
    color=1;
    Serial.println("Block is Red");
    Serial.print("H:\t");     Serial.print(hsv[0]);
    Serial.print("\tS:\t");   Serial.print(hsv[1]);
    Serial.print("\tV:\t");   Serial.println(hsv[2]);
  }
  else if ((hsv[0]>= 0.2) && (hsv[0]<= 0.6) && (hsv[1]>= 0.10) && (hsv[1]<= 0.30) && (hsv[2]>= 90.0) && (hsv[2]<= 105.0)){
    color=0;
    Serial.println("Block is Blue");
    Serial.print("H:\t");     Serial.print(hsv[0]);
    Serial.print("\tS:\t");   Serial.print(hsv[1]);
    Serial.print("\tV:\t");   Serial.println(hsv[2]);
  }
  else if ((hsv[0]>= 0.10) && (hsv[0]<= 0.25) && (hsv[1]>= 0.33) && (hsv[1]<= 0.60) && (hsv[2]>= 95.0) && (hsv[2]<= 115.0)){
    color=2;
    Serial.println("Block is Green");
    Serial.print("H:\t");     Serial.print(hsv[0]);
    Serial.print("\tS:\t");   Serial.print(hsv[1]);
    Serial.print("\tV:\t");   Serial.println(hsv[2]);
  }
  else {
    Serial.println("Failed color check");
    Serial.print("H:\t");     Serial.print(hsv[0]);
    Serial.print("\tS:\t");   Serial.print(hsv[1]);
    Serial.print("\tV:\t");   Serial.println(hsv[2]);
    gripper_state=true;
    gripper();
    gripper_state=false;
    gripper();
    

    check++;
    if (check < 2){
      checkcolor();
    }
    else{
      color=0;
    }
  }
}


void homeLA (){
  const float homingSpeedInMMPerSec = 25.0*4;
  const float maxHomingDistanceInMM = 720*4;   // since my lead-screw is 38cm long, should never move more than that
  const int directionTowardHome = 1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward

  stepperLA.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, maxHomingDistanceInMM, 23);

  stepperLA.setStepsPerMillimeter(40 * 4);
  stepperLA.setSpeedInMillimetersPerSecond(26.5);
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
  stepper2.moveToPositionInRevolutions(safelocation[height][1]/2.0);
  stepper1.moveToPositionInRevolutions(safelocation[height][0]/2.0);
  
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

  if (tcs.begin()) {
    Serial.println("Found color sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
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
  stepperLA.setSpeedInMillimetersPerSecond(29);
  stepperLA.setAccelerationInMillimetersPerSecondPerSecond(50.0);
  
  stepper1.setStepsPerRevolution(200*4); // times 8x microstepping
  stepper1.setSpeedInRevolutionsPerSecond(1.5);
  stepper1.setAccelerationInRevolutionsPerSecondPerSecond(1.0);

  stepper2.setStepsPerRevolution(200*8); // times 8x microstepping
  stepper2.setSpeedInRevolutionsPerSecond(1.5);
  stepper2.setAccelerationInRevolutionsPerSecondPerSecond(1.0);
  
  gripper_state=true;
  gripper();
  
  while(true){  
    buttons();
    if (LeftButton == false){
      for (int i = 0; i <= 9; i++) {

        if(i==9) { 
          while(true){}
        }

        // Open Gripper
        Serial.print("Loop Iteration: ");
        Serial.println(i);
        gripper_state=true;
        gripper();
        //Move to pick up location
        stepperLA.moveToPositionInMillimeters(zlocations[4]);

        if ((i == 7) || (i == 8)){
          stepper2.moveToPositionInRevolutions(startlocations[i][0]/2.0);
          stepper1.moveToPositionInRevolutions(startlocations[i][1]/2.0);
        }
        else {
          stepper1.moveToPositionInRevolutions(startlocations[i][0]/2.0);
          stepper2.moveToPositionInRevolutions(startlocations[i][1]/2.0);
        }
        
        if ((i == 6) || (i == 7) || (i == 8)){
          stepper1.moveToPositionInRevolutions(startlocations[i][0]);
          stepper2.moveToPositionInRevolutions(startlocations[i][1]);
        }
        else{
          stepper1.moveToPositionInRevolutions(startlocations[i][0]);
          stepper2.moveToPositionInRevolutions(startlocations[i][1]);
        }

        stepperLA.moveToPositionInMillimeters(zlocations[0]);
        
        if (i==0){
          while(true){
            buttons();
            if (RightButton == false){ break;}
          }
        }
        
        Serial.println("Pick up location");
        //wait
        delay(1000);
        //find color
        checkcolor();
        
        //close gripper
        gripper_state=false;
        gripper();
        //delay(1000);
        stepper2.moveRelativeInRevolutions(0.06);
        stepper1.moveRelativeInRevolutions(0.2);
        stepperLA.moveToPositionInMillimeters(zlocations[4]);
        
        if (color == 0){           // Blue
          Serial.println("Grabbed block is blue");
          height = b;
          Serial.print("Case height: ");
          Serial.println(height);

          //b=2;
          if(b!=2){
            stepper2.moveToPositionInRevolutions(safelocation[b][1]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[b][0]/2.0);
            stepper2.moveToPositionInRevolutions(safelocation[b][1]);
            stepper1.moveToPositionInRevolutions(safelocation[b][0]);
            Serial.println("Moved to safe location");
            //delay(1000);
          }
          else {
            //stepper2.moveToPositionInRevolutions(safelocation[b][1]/2.0);
            //Serial.println("Test 1");
            //delay(1000);
            stepper1.moveToPositionInRevolutions(safelocation[b][0]/2.0);
            //Serial.println("Test 2");
            //delay(1000);
            stepper1.moveToPositionInRevolutions(safelocation[b][0]);
            //Serial.println("Test 3");
            //delay(1000);
            stepper2.moveToPositionInRevolutions(safelocation[b][1]);
            //Serial.println("Test 4");
            //delay(1000);
            
          }
          
          stepperLA.moveToPositionInMillimeters(caselocations[blue][2]);

          stepper2.moveToPositionInRevolutions(caselocations[blue][1]);
          stepper1.moveToPositionInRevolutions(caselocations[blue][0]);

          gripper_state=true;
          gripper();
          
          
          stepper2.moveToPositionInRevolutions(safelocation[b][1]);
          stepper1.moveToPositionInRevolutions(safelocation[b][0]);
          
          blue++;
          b++;
        }
        else if (color == 1){      // Red
          Serial.println("Grabbed block is red");
          height = r;
          Serial.print("Case height: ");
          Serial.println(height);

          if(r!=2){
            stepper2.moveToPositionInRevolutions(safelocation[r][1]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[r][0]/2.0);
            stepper2.moveToPositionInRevolutions(safelocation[r][1]);
            stepper1.moveToPositionInRevolutions(safelocation[r][0]);
            Serial.println("Moved to safe location");
            //delay(1000);
            Serial.println("Moved to safe location");
            //delay(5000);
          }
          else {
            //stepper2.moveToPositionInRevolutions(safelocation[r][1]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[r][0]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[r][0]);
            stepper2.moveToPositionInRevolutions(safelocation[r][1]);
          }

          stepperLA.moveToPositionInMillimeters(caselocations[red][2]);

          stepper2.moveToPositionInRevolutions(caselocations[red][1]);
          stepper1.moveToPositionInRevolutions(caselocations[red][0]);

          gripper_state=true;
          gripper();
          
          stepper2.moveToPositionInRevolutions(safelocation[r][1]);
          stepper1.moveToPositionInRevolutions(safelocation[r][0]);
          
          red++;
          r++;
        }
        else if(color == 2){       // Green
          Serial.println("Grabbed block is green");
          height = g;
          Serial.print("Case height: ");
          Serial.println(height);
          
          if(g!=2){
            stepper2.moveToPositionInRevolutions(safelocation[g][1]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[g][0]/2.0);
            stepper2.moveToPositionInRevolutions(safelocation[g][1]);
            stepper1.moveToPositionInRevolutions(safelocation[g][0]);
            Serial.println("Moved to safe location");
            //delay(5000);
          }
          else {
            //stepper2.moveToPositionInRevolutions(safelocation[g][1]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[g][0]/2.0);
            stepper1.moveToPositionInRevolutions(safelocation[g][0]);
            stepper2.moveToPositionInRevolutions(safelocation[g][1]);
          }
          
          stepperLA.moveToPositionInMillimeters(caselocations[green][2]);

          stepper2.moveToPositionInRevolutions(caselocations[green][1]);
          stepper1.moveToPositionInRevolutions(caselocations[green][0]);

          gripper_state=true;
          gripper();
          
          
          stepper2.moveToPositionInRevolutions(safelocation[g][1]);
          stepper1.moveToPositionInRevolutions(safelocation[g][0]);
          
          green++;
          g++;
        }
      }
    }
  }
}


