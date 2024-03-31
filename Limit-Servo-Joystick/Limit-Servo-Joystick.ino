#include <Servo.h>
#include <ezButton.h>

ezButton limitSwitch(22);
Servo myservo;

int pos = 0;        // variable for servo position
int xPin = A1;      // variable for joystick pin
int yPin = A0;      // variable for joystick pin
int buttonPin = 2;  // variable for button pin

int xPosition = 0;      //position in the x for the joystick
int yPosition = 0;      //position in the y for the joystick
int buttonState = 0;    //position of the button for the joystick

void limit(){         // 
  limitSwitch.loop();

  if(limitSwitch.isReleased())
    Serial.println("The limit switch: UNTOUCHED -> TOUCHED");

  if(limitSwitch.isPressed())
    Serial.println("The limit switch: TOUCHED -> UNTOUCHED");

  int state = limitSwitch.getState();
  /*
  if(state == LOW)
    Serial.println("The limit switch: UNTOUCHED");
  else
    Serial.println("The limit switch: TOUCHED");
  */
}

void joystick() {
  xPosition = analogRead(xPin);
  yPosition = analogRead(yPin);
  buttonState = digitalRead(buttonPin);
  
  if(buttonState == 0){
    Serial.print("X: ");
    Serial.print(xPosition);
    Serial.print(" | Y: ");
    Serial.print(yPosition);
    Serial.print(" | Button: ");
    Serial.println(buttonState);
  }
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  limitSwitch.setDebounceTime(50);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { 
    myservo.write(pos);
    delay(15);
    limit();
    joystick();
    if (pos==0 || pos== 90 || pos == 180) {
      delay(750);
    }
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(15);
    limit();
    joystick();
    if (pos==0 || pos== 90 || pos == 180) {
      delay(750);
    }
  }
  //myservo.write(150);
}