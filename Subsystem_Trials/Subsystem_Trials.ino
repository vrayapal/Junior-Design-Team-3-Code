#include <Servo.h>
#include <ezButton.h>

ezButton limitSwitch1(7);
ezButton limitSwitch2(6);
ezButton limitSwitch3(5);

Servo myservo;

bool limit1 = false;
bool limit2 = false;
bool limit3 = false;

int gripper_pos = 0;        // variable for servo position
bool gripper_state = false;
int gripper_closed = 0;
int gripper_open = 90;



void limit(){               // limit switch function for limit switches. To add more add another limit do the following: add a block in this function; intialize 'limit4' above; set debounce time in setup

  limitSwitch1.loop();
  limit1 = limitSwitch1.getState();
    if(limit1 == true )
      limit1 = true;
      //code scoot robot away from limit switch 
    else if(limit1 == false)
      limit1 = false;
      
  limitSwitch2.loop();
  limit2 = limitSwitch2.getState();
    if(limit2 == true )
      limit2 = true;
      //code scoot robot away from limit switch 
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

void gripper(){ // function to actuate gripper from open to closed and vice versa. Requires values to be plugged in from 
  gripper_state = !gripper_state // False is open; True is closed
  if (gripper_state == true){
    for (pos = gripper_open; pos <= int gripper_closed; pos += 1) { 
      myservo.write(pos);
      delay(10)
      }
    }
  }
  if (gripper_state == false){
    for (pos = gripper_closed; pos <= int gripper_open; pos -= 1) { 
      myservo.write(pos);
      delay(10)
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  limitSwitch1.setDebounceTime(50);
  limitSwitch2.setDebounceTime(50);
  limitSwitch3.setDebounceTime(50);

}

void loop() {
  limit();
  Serial.print(limit1);
  Serial.print(limit2);
  Serial.println(limit3);

  // gripper()
}
