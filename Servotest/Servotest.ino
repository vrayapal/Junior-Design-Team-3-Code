#include <Servo.h>

Servo myservo;

int pos = 0;

void setup() {
  myservo.attach(9);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { 
    myservo.write(pos);
    delay(15);
    if (pos==0 || pos== 90 || pos == 180) {
      delay(750);
    }
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
  //myservo.write(150);
}