#include <Servo.h>
#include <ezButton.h>

ezButton limitSwitch1(23);
ezButton limitSwitch2(25);
ezButton limitSwitch3(27);

Servo gripper_servo;

//Limit Switch Variables
bool limit1 = false;
bool limit2 = false;
bool limit3 = false;

// Gripper Variables
int gripper_pos = 0;        // variable for servo position
bool gripper_state = false; // Initializes in the open state
int gripper_closed = 0;     // servo value at which the servo is closed 
int gripper_open = 90;      // servo value at which the servo is closed
int color = 0;              // Initialises variable to hold the color value fromt eh color sensor

//Stepper Test
int stepPin = 5; 
int dirPin = 4; 
int enPin = 6;
int StepperSpeed = 500;


void limit(){               // limit switch function for limit switches. To add more add another limit do the following: add a block in this function; intialize 'limit4' above; set debounce time in setup

  limitSwitch1.loop();
  limit1 = limitSwitch1.getState();
    if(limit1 == true)
      limit1 = true;
      //code to scoot robot away from limit switch 
    else if(limit1 == false)
      limit1 = false;
      
  limitSwitch2.loop();
  limit2 = limitSwitch2.getState();
    if(limit2 == true )
      limit2 = true;
      //code to scoot robot away from limit switch 
    else if(limit2 == false)
      limit2 = false;

  limitSwitch3.loop();
  limit3 = limitSwitch3.getState();
    if(limit3 == true )
      limit3 = true;
      //code scoot robot away from limit switch 
    else if(limit3 == false)
      limit3 = false;
}

void gripper(){                       // function to actuate gripper from open to closed and vice versa. Requires values to be plugged in from 
  gripper_state = !gripper_state;     // False is open; True is closed
  if (gripper_state == true)          // If gripper is going to close
  //Actuates the gripper from open to close position
    for (gripper_pos = gripper_open; gripper_pos <= gripper_closed; gripper_pos += 1) {  //change pos += 1 depending on if we are opening or closing servo
      gripper_servo.write(gripper_pos);
      delay(10);                      // increase if servo moves too fast and vice versa
      //check grip
      //find color of block
    }
  else if (gripper_state == false)    //If gripepr is going to open
  //Actuates the gripper from closed to open position
    for (gripper_pos = gripper_closed; gripper_pos <= gripper_open; gripper_pos -= 1) { //change pos -= 1 depending on if we are opening or closing servo
      gripper_servo.write(gripper_pos);
      delay(10);                      // increase if servo moves too fast and vice versa
      }
}

/*
void color (){
  if          ((myColor.getCIEx()>= 0) && (myColor.getCIEx()<= 1) && (myColor.getCIEy()>= 0) && (myColor.getCIEy()<= 1))
    color = 0;
    Serial.println("Block is Blue");
  else if     ((myColor.getCIEx()>= 2) && (myColor.getCIEx()<= 3) && (myColor.getCIEy()>= 2) && (myColor.getCIEy()<= 3))
    color = 1;
    Serial.println("Block is Green");
  else if     ((myColor.getCIEx()>= 4) && (myColor.getCIEx()<= 5) && (myColor.getCIEy()>= 4) && (myColor.getCIEy()<= 5))
    color = 2;
    Serial.println("Block is Red");
  else 
      Serial.println("Color not detected");
}
*/

void stepper1(){               // limit switch function for limit switches. To add more add another limit do the following: add a block in this function; intialize 'limit4' above; set debounce time in setup
  
}

void setup() {
  Serial.begin(9600);
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  limitSwitch3.setDebounceTime(50);

  //Color Sensor Setup 
  /*
  Serial.println("OPT4048 Example 1 Basic Color Sensing.");
  Wire.begin();
  if (!myColor.begin()) {
      Serial.println("OPT4048 not detected- check wiring or that your I2C address is correct!");
      while (1);
  }
    myColor.setBasicSetup();
    Serial.println("Ready to go!");
  */
}

void loop() {
  limit();
  Serial.print(limit1);
  Serial.print(limit2);
  Serial.println(limit3);

  // gripper()
  // exit()
}
