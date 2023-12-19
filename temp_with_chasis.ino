#include<LiquidCrystal.h>
#include<SoftwareSerial.h>
LiquidCrystal lcd(33, 35, 37, 39, 41, 43); //connecting the pins rs,en,d4,d5,d6,d7 to the arduino at pin 33 35 37 39 41 43 
int celsius; //declare a function celsius as an integer

#include <Servo.h>
const int SERVO  =7;   //Servo on Pin 9
const int IR     =A1;   //IR Distance Sensor on Analog Pin 0
int val = 0;            //IR test output

// Temperature Alert!
const int BLED=27;          // Blue LED Cathode on Pin 9
const int GLED=25;         // Green LED Cathode on Pin 10
const int RLED=23;         // Red LED Cathode on Pin 11
const int TEMP=A15;          // Temp Sensor is on pin A0
const int FAN=6;            // Fan pin 2
const int LOWER_BOUND=139; // Lower Threshold
const int UPPER_BOUND=149; // Upper Threshold
const int BTLE = 42;        //Sends analog data to BTLE device
int temperature = 0;               // Variable to hold analog reading

//Self-Driving Car - IR obstacle avoidance!
//H-Bridge Pins
const int RIGHT_EN  =8;  //Half Bridge Enable for Right Motor
const int RIGHT_MC1 =10;  //Right Bridge Switch 1 Control
const int RIGHT_MC2 =9;  //Right Bridge Switch 2 Control
const int LEFT_EN   =11; //Half Bridge Enable for Left Motor
const int LEFT_MC1  =13;  //Left Bridge Switch 1 Control
const int LEFT_MC2  =12;  //Left Bridge Switch 2 Control

// These constants won't change:
const int sensorPin = A1;  // pin that the sensor is attached to
const int ledPin = 9;      // pin that the LED is attached to

// variables:
int sensorValue = 0;   // the sensor value
int sensorMin = 1023;  // minimum sensor value
int sensorMax = 0;     // maximum sensor value

//Movement Thresholds and Speeds
const int SPEED_MIN = 150;            //Minimum motor speed
const int SPEED_MAX = 255;            //Maximum motor speed

int rate = 0;
Servo myServo;     //Servo Object

int readDistance(int pos)
{
    myServo.write(pos);                //Move to given position
    delay(1300);                       //Wait for Servo to move
    int dist = analogRead(IR);         //Read IR Sensor
    dist = map(dist, 50, 500, 0, 255); //scale it to LED range 
    dist = constrain(dist, 0, 255);    //Constrain it
    return dist;                       //Return scaled distance  
}

int LeftDistance, RightDistance, FrontDistance;

void setup()
{
   lcd.begin (16,2); //start LCD 16x2
   myServo.attach(SERVO); //Attach the Servo
   //The H-Bridge Pins are Outputs
    pinMode(RIGHT_EN, OUTPUT);
    pinMode(RIGHT_MC1, OUTPUT);
    pinMode(RIGHT_MC2, OUTPUT);
    pinMode(LEFT_EN, OUTPUT);
    pinMode(LEFT_MC1, OUTPUT);
    pinMode(LEFT_MC2, OUTPUT);
    pinMode (BLED, OUTPUT); // Set Blue LED as Output
    pinMode (GLED, OUTPUT); // Set Green LED as Output
    pinMode (RLED, OUTPUT); // Set Red LED as Output
    pinMode (FAN, OUTPUT);  // Set Fan as Output
    pinMode (BTLE, OUTPUT); //Sets BTLE as Output
    //Initialize with both motors stopped
    brake();
//Run a Serial interface for helping to calibrate the light levels.
    Serial.begin(9600);
// IR sensor calibration, without the LED:
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
    // signal the end of the calibration period
  digitalWrite(3, LOW);
// calibrate during the first five seconds
  while (millis() < 5000) {
    sensorValue = analogRead(sensorPin);

//record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }
    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }
}

void loop()
{
//read the sensor:
sensorValue = analogRead(sensorPin);
// in case the sensor value is outside the range seen during calibration
  sensorValue = constrain(sensorValue, sensorMin, sensorMax);
// apply the calibration to the sensor reading
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
// fade the LED using the calibrated value:
  analogWrite(ledPin, sensorValue);
  myServo.write (90);                                // Tells the Servo  to position at 80 degrees (Facing forward).
  delay (100);                                      //  Delay for 0.1s.
  
  FrontDistance = readDistance(90);                 // Measuring  the Distance in CM.
  if (FrontDistance < 500) {                            // If obstacle  found in 30cm.
    forward();
   rate = 100;
  }
  else if ((FrontDistance >= 350) && (FrontDistance < 250)) { // If obstacle  found between 30cm to 60cm.
   forward ();
   rate = 150;
    
  }
  else if ((FrontDistance >= 100) && (FrontDistance  < 25)) { // If obstacle found between 60cm to 90cm.
   forward();
   rate = 200;
   
  }
 else {                                          // If obstacle cannot be  found in 90cm.
   forward ();                                   // Robot move  to Forward direction.
   rate = 255;
   
}

RightDistance = readDistance(165);      // Measuring  the Distance in CM.
   if (RightDistance < 500) {                            // If obstacle  found in 30cm.
    forward();
    rate = 100;
  }
  else if ((RightDistance >= 350) && (RightDistance < 250)) { // If obstacle  found between 30cm to 60cm.
    forward ();
  rate = 150;
  }
  else if ((RightDistance >= 100) && (RightDistance  < 25)) { // If obstacle found between 60cm to 90cm.
    forward();
    rate = 200;
  }
  else {                                          // If obstacle cannot be  found in 90cm.
    forward ();                                   // Robot move  to Forward direction.
    rate = 255;
  }

 
  LeftDistance = readDistance(15);     // Measuring  the Distance in CM.
    if (LeftDistance < 500) {                            // If obstacle  found in 30cm.
    forward();
    
    rate = 100;
  }
 else if ((LeftDistance >= 350) && (LeftDistance < 250)) { // If obstacle  found between 30cm to 60cm.
    forward ();
    rate = 150;
  }
  else if ((LeftDistance >= 100) && (LeftDistance  < 25)) { // If obstacle found between 60cm to 90cm.
    forward();
    rate = 200;
  }
  else {                                          // If obstacle cannot be  found in 90cm.
    forward ();                                   // Robot move  to Forward direction.
    rate = 255;
  }
  temperature = analogRead(TEMP);
  val = analogRead(IR);
  Serial.print("Temperature ");
  Serial.println(temperature);
  Serial.print("IR distance  ");
  Serial.println(val);
  delay(500);

  // LED is Blue
  if (temperature < LOWER_BOUND)
  {
    digitalWrite(RLED, HIGH);
    digitalWrite(GLED, HIGH);
    digitalWrite(BLED, LOW);
    digitalWrite(FAN, LOW);
  }
  // LED is Red
  else if (temperature > UPPER_BOUND)
  {
    digitalWrite(RLED, LOW);
    digitalWrite(GLED, HIGH);
    digitalWrite(BLED, HIGH);
    digitalWrite(FAN, HIGH);
  }
  // LED is Green
  else
  {
    digitalWrite(RLED, HIGH);
    digitalWrite(GLED, LOW);
    digitalWrite(BLED, HIGH);
    digitalWrite(FAN, LOW);
  }
  int sensor_data = analogRead(TEMP);
  float voltage = sensor_data * 5.0;
  voltage/=1024;
  float temperatureC = (voltage - 0.5) * 100;
  delay(1000); 
  //Make custom characters
  lcd.setCursor(0,0);          
  lcd.print("Temperature"); 
  lcd.setCursor(0,1);           
  lcd.print(temperatureC);
  lcd.setCursor(5,1);
  lcd.print(char(223));
  lcd.print("C");
  lcd.print(" ");
  analogWrite(42,temperatureC); 
}
void CompareDistance() {
  if (LeftDistance > RightDistance)
 {TurnLeft (); }
 else if (RightDistance > LeftDistance)
 {TurnRight (); }
 else 
 { Reverse();
 TurnRight(); }
}

//Motor goes forward at given rate (from 0-255)
//Motor can be "left" or "right"
void forward ()
{ 
 digitalWrite(LEFT_EN, HIGH);
 digitalWrite(LEFT_MC1, HIGH);
 digitalWrite(LEFT_MC2, LOW);
 //analogWrite(LEFT_EN, rate);
 digitalWrite(RIGHT_EN, HIGH);
 digitalWrite(RIGHT_MC1, HIGH);
 digitalWrite(RIGHT_MC2, LOW);
 //analogWrite(RIGHT_EN, rate);
}
void TurnLeft ()
{ //right forward left reverse
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(LEFT_MC1, LOW);
  digitalWrite(LEFT_MC2, HIGH);
  //analogWrite(LEFT_EN, rate);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(RIGHT_MC1, HIGH);
  digitalWrite(RIGHT_MC2, LOW);
  //analogWrite(RIGHT_EN, rate);
}
void TurnRight ()
{ //left forward right reverse
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(RIGHT_MC1, LOW);
  digitalWrite(RIGHT_MC2, HIGH);
  //analogWrite(RIGHT_EN, rate);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(LEFT_MC1, HIGH);
  digitalWrite(LEFT_MC2, LOW);
  //analogWrite(LEFT_EN, rate);
}
void Reverse()
{ 
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(LEFT_MC1, LOW);
  digitalWrite(LEFT_MC2, HIGH);
  //digitalWrite(LEFT_EN, rate);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(RIGHT_MC1, LOW);
  digitalWrite(RIGHT_MC2, HIGH);
  //digitalWrite(RIGHT_EN, rate);
}
void brake()
{
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(LEFT_MC1, LOW);
  digitalWrite(LEFT_MC2, LOW);
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(RIGHT_MC1, LOW);
  digitalWrite(RIGHT_MC2, LOW);
  digitalWrite(RIGHT_EN, HIGH);
}
