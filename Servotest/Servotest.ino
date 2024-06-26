#include <Servo.h>

Servo myservo;

int pos = 60;

int max = 120;
;
int min = 30;
int speed = 8; // smaller is faster

void setup() {
  Serial.begin(9600);
  myservo.attach(12); //PWM
}

void loop() {
  for (pos = min; pos <= max; pos += 1) { 
    myservo.write(pos);
    Serial.println(pos);
    delay(speed);
    }
  for (pos = max; pos >= min; pos -= 1) {
    myservo.write(pos);
    Serial.println(pos);
    delay(speed);
    if(pos==min)
      delay(2000);
  }
  //myservo.write(70);
}