#include <Servo.h>

Servo myservo;

int pos = 100;

int max = 80;
int min = -20;
int speed = 15; // smaller is faster

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
  //myservo.write(150);
}