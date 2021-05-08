#include <Adafruit_NeoPixel.h>          // Library for NeoPixels
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position
int max_angle = 115;//maximum angle for servo
int min_angle = 9.5;//minimum angle for servo
#define steps 200;//servo steps
int servoPin = 3;

//Outer LEDs
#define pinPix 2                       // Pin driving NeoPixel Ring or String
#define numPix 7                      // Number of NeoPixels in the Ring or Strip
#define flash 6                      //Number of times for eye to flash
#define R 0
#define G 0
#define B 140
Adafruit_NeoPixel myNeoPixels = Adafruit_NeoPixel(numPix, pinPix, NEO_GRB + NEO_KHZ800);//Sets up pixels


//Inner LEDs
#define pinPix_center 4
#define numPix_center 2
#define R_c 35
#define G_c 35
#define B_c 70
Adafruit_NeoPixel myNeoPixels_center = Adafruit_NeoPixel(numPix_center, pinPix_center, NEO_GRB + NEO_KHZ800);//Sets up pixels

//Lid Opening Button
#define LEDPIN 13         // Board LED pin
#define buttonPin 0 //number of button pin
int buttonState = 0;//variable for reading pushbutton status
int openState = 0;//variable to determine if box is open or closed
int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 20;   // the debounce time, increase if the output flickers
long time_lid = 0; //the last time the lid moved

//Setup
void setup() {
  myNeoPixels.begin();   // Initialize the NeoPixel array in the Arduino's memory,
  myNeoPixels.show();    // turn all pixels off, and upload to ring or string

  myNeoPixels_center.begin();   // Initialize the NeoPixel array in the Arduino's memory,
  myNeoPixels_center.show();    // turn all pixels off, and upload to ring or string
  
  pinMode(LEDPIN, OUTPUT); 
  pinMode(buttonPin, INPUT); 
  digitalWrite(buttonPin, LOW);
  Serial.begin(9600);
  myservo.attach(servoPin);
}


//Loop
void loop() {
  //breathe(20, 100, 255,39,0);       // Pause; Number of steps; Full-on R,G,B values
  // read the state of the pushbutton
  buttonState = digitalRead(buttonPin);
  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (buttonState == HIGH && millis() - time > debounce) {
    if (state == HIGH)
      state = LOW;
    else
      state = HIGH;

    time = millis();    
  }
  digitalWrite(LEDPIN, state);

  if(state==HIGH && openState == 0){
    //turn NeoPixels on:
    for (int i=0;i<numPix;i++){
      myNeoPixels.setPixelColor(i,R,G,B);
    }
    myNeoPixels.show();

    for (int i=0;i<numPix_center;i++){
      myNeoPixels_center.setPixelColor(i,R_c,G_c,B_c);
    }
    myNeoPixels_center.show(); 
    //Open Lid 
    myservo.attach(servoPin);
    for (pos = 3; pos <= max_angle; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    openState = 1;
    myservo.detach();
  }else if(state==LOW&&openState==1){
    //turn off Neopixels
    myNeoPixels_center.clear(); 
    myNeoPixels.clear();
    myNeoPixels_center.show(); 
    myNeoPixels.show();
    //Close Lid
    myservo.attach(servoPin);
    for (pos = 115; pos >= min_angle; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    openState = 0; 
    myservo.detach();
  
  }

  previous = buttonState;

}
