int xPin = A3;      // variable for joystick pin  (digital)
int yPin = A2;      // variable for joystick pin  (digital)
int buttonPin = 3;  // variable for button pin (analog in) SW

double xPosition = 0;      //position in the x for the joystick
double yPosition = 0;      //position in the y for the joystick
int buttonState = 0;    //position of the button for the joystick

void setup() {
  Serial.begin(9600); 
  
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  pinMode(buttonPin, INPUT_PULLUP); 
}

void loop() {
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
    delay(1000);
  }


  delay(100); // add some delay between reads
}