#include <ezButton.h>

ezButton limitSwitch(22);

void setup() {
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);
}

void loop() {
  limitSwitch.loop();

  if(limitSwitch.isReleased())
    Serial.println("The limit switch: UNTOUCHED -> TOUCHED");

  if(limitSwitch.isPressed())
    Serial.println("The limit switch: TOUCHED -> UNTOUCHED");

  int state = limitSwitch.getState();
  if(state == LOW)
    Serial.println("The limit switch: UNTOUCHED");
  else
    Serial.println("The limit switch: TOUCHED");
}
