#include "Wire.h"
#include "Adafruit_TCS34725.h"

 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

double hue, saturation, lighting, value;

float fract(float x) { return x - int(x); }

float mix(float a, float b, float t) { return a + (b - a) * t; }

float step(float e, float x) { return x < e ? 0.0 : 1.0; }

float hsv[3];

float* rgb2hsv(float r, float g, float b, float* hsv) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  hsv[0] = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  hsv[1] = d / (qx + 1e-10);
  hsv[2] = qx;
  return hsv;
}

void setup() {
  Serial.begin(9600);
  //Serial.println("Color View Test!");
 
  if (tcs.begin()) {
    Serial.println("Found color sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}
 
void loop() {
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
 
  //Serial.print("R:\t"); Serial.print(int(red));
  //Serial.print("\tG:\t"); Serial.print(int(green));
  //Serial.print("\tB:\t"); Serial.print(int(blue));

  rgb2hsv(red, green, blue, hsv);
  
  Serial.print("H:\t");     Serial.print(hsv[0]);
  Serial.print("\tS:\t");   Serial.print(hsv[1]);
  Serial.print("\tV:\t");   Serial.println(hsv[2]);
  
  int color=0;

  if ((hsv[0]>= 0.0) && (hsv[0]<= 0.1) && (hsv[1]>= 0.36) && (hsv[1]<= 0.80) && (hsv[2]>= 100.0) && (hsv[2]<= 190.0)){
    color=1;
    Serial.println("Block is Red");
  }
  else if ((hsv[0]>= 0.2) && (hsv[0]<= 0.6) && (hsv[1]>= 0.10) && (hsv[1]<= 0.30) && (hsv[2]>= 90.0) && (hsv[2]<= 98.0)){
    color=0;
    Serial.println("Block is Blue");
  }
  else if ((hsv[0]>= 0.10) && (hsv[0]<= 0.25) && (hsv[1]>= 0.33) && (hsv[1]<= 0.60) && (hsv[2]>= 95.0) && (hsv[2]<= 115.0)){
    color=2;
    Serial.println("Block is Green");
  }
  else {
    Serial.println("Failed color check");
  }
}