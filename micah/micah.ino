int stepPin = 5; 
int dirPin = 4; 
int enPin = 6;
int StepperSpeed = 500;


void setup() {
  Serial.begin(9600);
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  
}

void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  for(int x = 0; x < 800; x++) {
    Serial.println("High");
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(StepperSpeed);
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(StepperSpeed); 
  }
  delay(1000); // One second delay
  digitalWrite(dirPin,LOW); //Changes the direction of rotation
  for(int x = 0; x < 800; x++) {
    Serial.println("Low");
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(StepperSpeed);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(StepperSpeed);
  }
  delay(1000); 
}