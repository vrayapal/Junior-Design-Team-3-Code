
#include "SparkFun_OPT4048.h"
#include <Wire.h>

SparkFun_OPT4048 myColor;
int color = 0;

void print(){
  Serial.print("CIEx: ");
    Serial.print(myColor.getCIEx());
    Serial.print(" CIEy: ");
    Serial.println(myColor.getCIEy());
    // Delay time is set to the conversion time * number of channels
    // You need three channels for color sensing @ 200ms conversion time = 600ms.
    delay(200);
}


void setup(){
    Serial.begin(9600);
    Serial.println("OPT4048 Example 1 Basic Color Sensing.");

    Wire.begin();

    if (!myColor.begin()) {
        Serial.println("OPT4048 not detected- check wiring or that your I2C address is correct!");
        while (1) ;
    }

    myColor.setBasicSetup();

    Serial.println("Ready to go!");
}


void loop()
{
  /*
  Serial.print("CIEx: ");
  Serial.print(myColor.getCIEx());
  Serial.print(" CIEy: ");
  Serial.println(myColor.getCIEy());
  // Delay time is set to the conversion time * number of channels
  // You need three channels for color sensing @ 200ms conversion time = 600ms.
  delay(200);
  */
  if ((myColor.getCIEy()>= 0.20) && (myColor.getCIEy()<= 0.35) && (myColor.getCIEx()>= 0.35) && (myColor.getCIEx()<= 0.50)){
    color=0;
    Serial.println("Block is Blue");
    print();
  }
  else if ((myColor.getCIEx()>= 0.55) && (myColor.getCIEx()<= 0.70) && (myColor.getCIEy()>= 0.25) && (myColor.getCIEy()<= 0.35)){
    color=1;
    Serial.println("Block is Red");
    print();
  }
  else if ((myColor.getCIEx()>= 0.35) && (myColor.getCIEx()<= 0.55) && (myColor.getCIEy()>= 0.36) && (myColor.getCIEy()<= 0.50)){
    color=2;
    Serial.println("Block is Green");
  }
  else {
    print();
    }
    
}
