#include <Servo.h>
#include <ezButton.h>

ezButton limitSwitch(22);
Servo myservo;

int pos = 0;

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

void setup() {
  myservo.attach(9);
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { 
    myservo.write(pos);
    delay(15);
    limit();
    if (pos==0 || pos== 90 || pos == 180) {
      delay(750);
    }
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(15);
    limit();
  }
  //myservo.write(150);
}