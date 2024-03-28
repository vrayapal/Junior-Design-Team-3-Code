#include <Servo.h>
#include <ezButton.h>

ezButton limitSwitch(22);
Servo myservo;

int pos = 0;
int xPin = A1;
int yPin = A0;
int buttonPin = 2;

int xPosition = 0;
int yPosition = 0;
int buttonState = 0;

void limit(){
  limitSwitch.loop();

  if(limitSwitch.isReleased())
    Serial.println("The limit switch: UNTOUCHED -> TOUCHED");

  if(limitSwitch.isPressed())
    Serial.println("The limit switch: TOUCHED -> UNTOUCHED");

  int state = limitSwitch.getState();
  /*if(state == LOW)
    Serial.println("The limit switch: UNTOUCHED");
  else
    Serial.println("The limit switch: TOUCHED");*/
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