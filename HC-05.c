#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'

#include <Arduino.h>
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
const int irSensorPin1 = A3; // Left IR Sensor
const int irSensorPin2 = A0; // Right IR Sensor

// Variables for pulse counts
volatile unsigned long pulses1 = 0; // Total pulses for encoder 1
volatile unsigned long pulses2 = 0; // Total pulses for encoder 2
const unsigned int pulsesperturn = 20; // Pulses per revolution

// Wheel and distance calculations
const float wheelDiameter = 6.2; // Diameter of the wheel in cm
const float wheelCircumference = 3.14 * wheelDiameter; // Circumference of the wheel in cm

// Time tracking
unsigned long startTime; // Store the time the program starts
unsigned long elapsedTime;

// Interrupt service routines
void encoder1ISR() {
  pulses1++;
}

void encoder2ISR() {
  pulses2++;
}

void setup() {
  Serial.begin(9600);  // Set the baud rate for serial communication

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

void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      MoveForward(255);  // Move forward at full speed
      break;
    case BACKWARD:
      // Perform action for moving backward (you need to define this function)
      break;
    case LEFT:
      Left();  // Turn left
      break;
    case RIGHT:
      Right();  // Turn right
      break;
    case CIRCLE:
      // Perform action for circle (you need to define this function)
      break;
    case CROSS:
      Stop();  // Stop immediately
      break;
    case TRIANGLE:
      // Perform action for toggling a state (e.g., LED on/off)
      break;
    case SQUARE:
      // Perform action for retrieving and sending status information
      break;
    case START:
      // Perform action for starting a process or operation
      break;
    case PAUSE:
      // Perform action for pausing a process or operation
      break;
    default:
      // Invalid command received
      break;
  }
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);  // Execute the appropriate command based on serial input
  }

  // Continue with other tasks in your main loop, such as updating the LCD or checking sensors
}