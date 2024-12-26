#include <LiquidCrystal.h>
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#define SDA A4
#define SCL A5

#define RAD_TO_DEG 57.2958
#define PI 3.14159265359

// Motor Pins
const int IN1 = 2;
const int IN2 = 10;
const int IN3 = 12;
const int IN4 = 13;
const int FNA = 3;
const int FNB = 11;

// Encoder pin definitions
const int encoder_pin1 = A1; // Right Encoder 
const int encoder_pin2 = A2; // Left Encoder 

// IR sensor pins
const int irSensorPin1 = A3; // Left IR Sensor
const int irSensorPin2 = A0; // Right IR sensor

// MPU6050 Constants
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ;
float y;          // Y-axis angle in degrees
float yOffset = 0; // Calibration offset

// Calibration and smoothing constants
const int numSamples = 20;
float ySamples[numSamples];
int sampleIndex = 0;

// LCD Pins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Setup MPU6050
void setupMPU6050() {
  Wire.begin();
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0); // Wake up MPU6050
  Wire.endTransmission(true);

  // Collect baseline calibration offset
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    readMPU6050Raw();
    sum += RAD_TO_DEG * atan2(AcX, AcZ);
    delay(10);
  }
  yOffset = sum / 50.0;
}

// Read raw MPU6050 data
void readMPU6050Raw() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
}

// Read and smooth Y-axis angle from MPU6050
void readMPU6050() {
  readMPU6050Raw();
  float rawY = RAD_TO_DEG * atan2(AcX, AcZ) - yOffset;
  
  ySamples[sampleIndex] = rawY;
  sampleIndex = (sampleIndex + 1) % numSamples;

  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += ySamples[i];
  }
  y = sum / numSamples;
}

// Update LCD with Y-axis angle
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Y-axis angle: ");
  lcd.print(y, 2); // Display Y-axis angle with 2 decimal places
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

  // Initialize components
  setupMPU6050();
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  Serial.begin(9600); // Start Serial Communication
  Serial.println("Live Y-axis Feed:");
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

bool checkForwardCondition() {
  int sensorValue1 = !digitalRead(irSensorPin1);
  int sensorValue2 = !digitalRead(irSensorPin2);
  return (sensorValue1 == 1 && sensorValue2 == 1);
}

void LineFollow(int PWM) {
  int sensorValue1 = !digitalRead(irSensorPin1);
  int sensorValue2 = !digitalRead(irSensorPin2);
  int Balance = sensorValue1 - sensorValue2;

  if (sensorValue1 == 0 && sensorValue2 == 0){
    //Display on 1st row: Line FWD
    MoveForward(PWM);
  }

  else if (Balance == -1){
    //Display on 1st row: Line R
    checkStopCondition();
    Right();
  }

   else if (Balance == 1){
    //Display on 1st row: Line L
    checkStopCondition();
    Left();
   }

   else if (sensorValue1 == 1 && sensorValue2 == 1) {
    //Display on 1st row: Line Stop 
    Stop();
   } 
}

void LineFollowUP(int PWM){
  int sensorValue1 = !digitalRead(irSensorPin1);
  int sensorValue2 = !digitalRead(irSensorPin2);
  int Balance = sensorValue1 - sensorValue2;

  if (sensorValue1 == 0 && sensorValue2 == 0){
    MoveForward(PWM);
    //Display on 1st row: Forward 
  }

  else if (Balance == -1){
    //Display on 1st row: Right 
    setMotorSpeed(FNA, 100);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    setMotorSpeed(FNB, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    //Tank Right 
    // setMotorSpeed(FNA, 100);
    // digitalWrite(IN1, HIGH);
    // digitalWrite(IN2, LOW);
    // setMotorSpeed(FNB, 255);
    // digitalWrite(IN3, HIGH);
    // digitalWrite(IN4, LOW);
  }

  else if (Balance == 1){
    //Display on 1st row: Left
  setMotorSpeed(FNA, 255);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  setMotorSpeed(FNB, 100);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //Tank Left 
  // setMotorSpeed(FNA, 255);
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // setMotorSpeed(FNB, 100);
  // digitalWrite(IN3, LOW);
  // digitalWrite(IN4, HIGH);
   }

   else if (sensorValue1 == 1 && sensorValue2 == 1) {
    //Display on 1st row: Forward 
    MoveForward(PWM);
    readMPU6050(); // Read Y-axis angle from MPU6050
   } 

}

void LineFollowDOWN(int PWM){
  int sensorValue1 = !digitalRead(irSensorPin1);
  int sensorValue2 = !digitalRead(irSensorPin2);
  int Balance = sensorValue1 - sensorValue2;

  if (sensorValue1 == 0 && sensorValue2 == 0){
     //Display on first row: Down FWD
    MoveForward(PWM);
  }

  else if (Balance == -1){
  checkForwardCondition();
  //Display on first row: Down R
  setMotorSpeed(FNA, 100);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setMotorSpeed(FNB, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  }

   else if (Balance == 1){
  checkForwardCondition();
 //Display on first row: Down L
  setMotorSpeed(FNA, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(FNB, 100);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
   }

     else if (sensorValue1 == 1 && sensorValue2 == 1) {
     //Display on first row: Down FWD
    MoveForward(PWM);
   }
} 

void Circle() {
   int rightSensorOffCount = 0; // Counter for right sensor being off
   
   for (;;) { // Infinite loop for rotation
     lcd.setCursor(0, 1);
     lcd.print("Circle R        ");
     
     Right();
     
     // Check if the right sensor is off
     if (!digitalRead(irSensorPin2)) {
       // Wait until the sensor becomes "on" again to avoid duplicate counting
       while (!digitalRead(irSensorPin2)) {
         // Do nothing and wait for the sensor to switch back to "on"
       }
       rightSensorOffCount++; // Increment the counter when the sensor transitions off
     }
     
     // Break the loop when the sensor is off twice
     if (rightSensorOffCount >= 2) {
       break;
     }
   }
   Right();
   delay(40);
   Stop();
   lcd.setCursor(0, 1);
   lcd.print("360 Done      ");
   delay(3000);
}

void Decending(){
   MoveForward(100);
   delay(1050);
   Stop();
   lcd.setCursor(0, 1);
   lcd.print("Decent done");
   delay(3000);
   
}

unsigned long previousMillis = 0;  // Stores the last time the angle was checked
const long interval = 2000;         // 2 seconds (2000 milliseconds)
bool inClimbingMode = false;        // Flag to check if the system is in climbing mode
// Variable to track if the robot has climbed and reached the peak
bool hasClimbed = false;
bool descentComplete = false;       // Added descent complete flag globally
bool peakReached = false; // Flag to ensure the peak function runs only once

void loop() {
   readMPU6050(); // Read Y-axis angle from MPU6050
   
   // Check and update the LCD display based on the climbing state
   if (!hasClimbed && inClimbingMode && y >= -21 && y <= -1) {
     lcd.setCursor(0, 0);
     lcd.print("Climbing: ");
     lcd.print(y, 2); // Live angle feed during climbing
   } else {
     updateLCD(); // Default behavior for updating LCD
   }

   // Display the angle on the Serial Monitor
   Serial.print("Y-axis angle: ");
   Serial.println(y, 2);
   
   // Only attempt to climb if not already climbed and peak not reached
   if (!hasClimbed && y >= -21 && y <= -1) {
     if (!inClimbingMode) {
       previousMillis = millis();  // Start the timer when entering the valid angle range
       inClimbingMode = true;
     }
     
     // Check if the angle has been within the range for longer than 2 seconds
     if (millis() - previousMillis >= interval) {
       // Enter climbing mode
       lcd.setCursor(0, 1);
       lcd.print("Climbing        ");
       
       // Climbing phase
       while (y <= -1 && !hasClimbed) {
         LineFollowUP(255); // Ramp climbing with higher speed
         delay(10);
         readMPU6050();   // Continuously update the angle during the climb

         // Update the live feed of the angle
         lcd.setCursor(0, 0);
         lcd.print("Climbing: ");
         lcd.print(y, 2); 
       }
       
       // Reached the peak
       lcd.setCursor(0, 1);
       lcd.print("Peak            ");
       
       LineFollow(65);  // Run line-following behavior after reaching the peak
       delay(40);
       
       Stop();           // Stop at the middle of the peak
       delay(4000);      // Pause for 4 seconds
       
       // Execute LineFollow after the peak
       lcd.setCursor(0, 1);
       lcd.print("Following Line  ");
       
       Circle(); // Execute 360 rotation
       
       // After 360 is done and descending
       Decending();
       hasClimbed = true;
       peakReached = true; // Mark the peak functionality as completed
     }
   }
   else {
     // Determine cruising behavior based on whether the robot has climbed
     if (!hasClimbed) {
       lcd.setCursor(0, 1);
       lcd.print("Cruising        ");
       LineFollow(80);            // Default forward movement before climbing
     } 
     else {
       lcd.setCursor(0, 1);
       lcd.print("Distance f(x)   ");
       LineFollow(67);             // Line-following behavior after climbing
       delay(1950);
       Stop();
       delay(3000):
       LineFollow(67);
     }
   }
   delay(100); // Optional: Adjust delay for better responsiveness
}

