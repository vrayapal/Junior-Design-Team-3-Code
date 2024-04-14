#include <ezButton.h>

ezButton limitSwitch(25);

bool limit = false;


void setup() {
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50);
}

void loop() {
  limitSwitch.loop();

  limit = limitSwitch.getState();
    if(limit == true){
      limit = true;
      Serial.println(limit);
    }
    else if(limit == false){
      limit = false;
      Serial.println(limit);

      //delay(100);
    }
}
