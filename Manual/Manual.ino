#include <Servo.h>
#include <ezButton.h>

ezButton limitSwitch1(7);
ezButton limitSwitch2(6);
ezButton limitSwitch3(5);

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

// Joystick 1 Variables
int x1Pin = A1;      // variable for joystick pin
int y1Pin = A0;      // variable for joystick pin
int buttonPin1 = 2;  // variable for button pin

float xPosition1 = 0;      //position in the x for the joystick
float yPosition1 = 0;      //position in the y for the joystick
int buttonState1 = 0;    //position of the button for the joystick

// Joystick 2 Variables
int x2Pin = A3;      // variable for joystick pin
int y2Pin = A2;      // variable for joystick pin
int buttonPin2 = 2;  // variable for button pin

float xPosition2 = 0;      //position in the x for the joystick
float yPosition2 = 0;      //position in the y for the joystick
int buttonState2 = 0;    //position of the button for the joystick



void joystick() {
  xPosition1 = (((analogRead(x1Pin))-511)/512);   // normalized x position from -1 to 1 for joystick 1
  yPosition1 = (((analogRead(y1Pin))-511)/512);   // normalized y position from -1 to 1 for joystick 1
  buttonState1 = digitalRead(buttonPin1);
  
  xPosition2 = (((analogRead(x2Pin))-511)/512);   // normalized x position from -1 to 1 for joystick 2
  yPosition2 = (((analogRead(y2Pin))-511)/512);   // normalized y position from -1 to 1 for joystick 2
  buttonState2 = digitalRead(buttonPin2);
  
  /* Debug Stuff
  if(buttonState == 0){
    Serial.print("X: ");
    Serial.print(xPosition);
    Serial.print(" | Y: ");
    Serial.print(yPosition);
    Serial.print(" | Button: ");
    Serial.println(buttonState);
  }
  */
}

void limit(){               // limit switch function for limit switches. To add more add another limit do the following: add a block in this function; intialize 'limit4' above; set debounce time in setup

  limitSwitch1.loop();
  limit1 = limitSwitch1.getState();
    if(limit1 == true )
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
  if ((colorsensorvalue>= 0) && (colorsensorvalue<= 1))
    color=0;
    Serial.println("Block is Blue");
  else if ((colorsensorvalue>= 2) && (colorsensorvalue<= 3))
    color=1;
    Serial.println("Block is Green");
  else if ((colorsensorvalue>= 4) && (colorsensorvalue<= 5))
    color=2;
    Serial.println("Block is Red");
  else 
      Serial.println("Color not detected");
}
*/


void setup() {
  Serial.begin(9600);
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  limitSwitch3.setDebounceTime(50);

}

void loop() {
  limit();
  /* Debug for limit switchs
  Serial.print(limit1);
  Serial.print(limit2);
  Serial.println(limit3);
  */
  joystick()
  if(buttonState1 == 0){
    Serial.print("X1: ");
    Serial.print(xPosition1);
    Serial.print(" | Y1: ");
    Serial.print(yPosition1);
    Serial.print(" | Button1: ");
    Serial.println(buttonState1);
  }
  if(buttonState2 == 0){
    Serial.print("X2: ");
    Serial.print(xPosition2);
    Serial.print(" | Y2: ");
    Serial.print(yPosition2);
    Serial.print(" | Button2: ");
    Serial.println(buttonState2);
  }
  // gripper()
  // exit()
}

