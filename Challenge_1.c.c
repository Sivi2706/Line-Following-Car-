#include<Arduino.h>
#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>

// Motor Pins
const int IN1 = 2;
const int IN2 = 10;
const int IN3 = 12;
const int IN4 = 13;
const int FNA = 3;
const int FNB = 11;

// Define the LCD pins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Encoder pin definitions
const int encoder_pin1 = A1; // Right Encoder 
const int encoder_pin2 = A2; // Left Encoder 

// IR sensor pins
const int irSensorPin1 = A3; //Left IR Sensor
const int irSensorPin2 = A0; //Right IR sensor

// Variables for pulse counts
volatile unsigned long pulses1 = 0; // Total pulses for encoder 1
volatile unsigned long pulses2 = 0; // Total pulses for encoder 2
const unsigned int pulsesperturn = 20; // Pulses per revolution

// Wheel and distance calculations
const float wheelDiameter  = 6.2;                      // Diameter of the wheel in cm
const float wheelCircumference = 3.14 * wheelDiameter; // Circumference of the wheel in cm

// Time tracking
unsigned long startTime;  // Store the time the program starts
unsigned long elapsedTime;

// Interrupt service routines
void encoder1ISR() {
  pulses1++;
}

void encoder2ISR() {
  pulses2++;
}

void setup() {
  // Set the output pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(FNA, OUTPUT);
  pinMode(FNB, OUTPUT);
  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);
  pinMode(encoder_pin1, INPUT_PULLUP);
  pinMode(encoder_pin2, INPUT_PULLUP);

  // Initialize interrupts for encoders
  attachPCINT(digitalPinToPCINT(encoder_pin1), encoder1ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(encoder_pin2), encoder2ISR, CHANGE);

  // Initialize the LCD
  lcd.begin(16, 2); // 16 columns, 2 rows
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  // Start time tracking
  startTime = millis();
}

void setMotorSpeed(int motorPWM, int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(motorPWM, speed);
}

void MoveForward(int PWM) {
  setMotorSpeed(FNA, PWM);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(FNB, PWM);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void Left() {
  setMotorSpeed(FNA, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(FNB, 255);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Right() {
  setMotorSpeed(FNA, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setMotorSpeed(FNB, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

bool checkStopCondition() {
  int sensorValue1 = !digitalRead(irSensorPin1);
  int sensorValue2 = !digitalRead(irSensorPin2);
  return (sensorValue1 == 1 && sensorValue2 == 1);
}

void loop() {
  // Calculate elapsed time
  elapsedTime = millis() - startTime; // Time in milliseconds

  // Variables to hold total distance
  static float distance1 = 0;
  static float distance2 = 0;

  // Read IR sensor values
  int sensorValue1 = !digitalRead(irSensorPin1);
  int sensorValue2 = !digitalRead(irSensorPin2);

  // Calculate the Balance value
  int Balance = sensorValue1 - sensorValue2;

  String currentDisplay;

  // Direction control logic
  if (sensorValue1 == 0 && sensorValue2 == 0) {
    //currentDisplay = "Forward";
    MoveForward(65);

    // Calculate total distance only in forward movement
    //distance1 += (pulses1 / (float)pulsesperturn) * wheelCircumference;
   // distance2 += (pulses2 / (float)pulsesperturn) * wheelCircumference;
    //pulses1 = 0; // Reset pulses after counting
    //pulses2 = 0; // Reset pulses after counting
  } 
  else if (Balance == -1) {
    currentDisplay = "Right";
    Stop();
    delay(10);
    Right();
    while (Balance == -1) {
      if (checkStopCondition()) { // Check if both IR sensors are 1
        currentDisplay = "Stop";
        Stop();
        break; // Exit the loop
      }
      sensorValue1 = !digitalRead(irSensorPin1);
      sensorValue2 = !digitalRead(irSensorPin2);
      Balance = sensorValue1 - sensorValue2;
      delay(20);
    }
  } 
  else if (Balance == 1) {
    currentDisplay = "Left";
    Stop();
    delay(10);
    Left();
    while (Balance == 1) {
      if (checkStopCondition()) { // Check if both IR sensors are 1
        currentDisplay = "Stop";
        Stop();
        break; // Exit the loop
      }
      sensorValue1 = !digitalRead(irSensorPin1);
      sensorValue2 = !digitalRead(irSensorPin2);
      Balance = sensorValue1 - sensorValue2;
      delay(20);
    }
  } 
  else if (sensorValue1 == 1 && sensorValue2 == 1) {
    currentDisplay = "Stop";
    Stop();
  } 
  else {
    currentDisplay = "Unstable Input";
    Stop();
  }

  // Calculate total distance and average it
  float totalDistance = (distance1 + distance2) / 2.0; // Average distance

  // Convert time to seconds
  float timeInSeconds = elapsedTime / 1000.0;

  // Update LCD with distance, time, and current direction
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(totalDistance, 2); // Display distance with 2 decimal places
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(timeInSeconds, 2); // Display time in seconds
  lcd.setCursor(0, 1); // Add direction display
  lcd.print("Dir: ");
  lcd.print(currentDisplay);

  delay(50); // Update every 50ms
}
