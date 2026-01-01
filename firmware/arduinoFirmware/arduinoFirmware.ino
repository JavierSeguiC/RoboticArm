#include <Servo.h>
#include <EEPROM.h>

// --- Configuration ---
Servo baseServo;
Servo gripperServo;

// Pins
const int BASE_PIN = 9;
const int GRIPPER_PIN = 6;

// EEPROM Addresses
const int ADDR_BASE = 0;

// DC Motor 1: ELBOW
const int ELBOW_PWM = 3;  // Speed pin
const int ELBOW_IN1 = 4;  // Direction 1
const int ELBOW_IN2 = 5;  // Direction 2
const int ELBOW_POT = A0; // Potentiometer

// DC Motor 2: WRIST
const int WRIST_PWM = 10;
const int WRIST_IN1 = 7; 
const int WRIST_IN2 = 8;
const int WRIST_POT = A1;

// Base Servo (Ramped Speed Logic)
int baseTarget = 90;
float baseCurrent = 90.0;
int baseStepDelay = 0; // Delay in ms (0 = Fast, Higher = Slower)
unsigned long lastBaseTime = 0;

// Elbow DC (PID Logic)
int elbowTarget = 90;
int elbowMaxPWM = 255; // Speed limit
double elbowKp = 4.0, elbowKd = 0.5; // Tuning constants
int lastElbowError = 0;

// Wrist DC (PID Logic)
int wristTarget = 90;
int wristMaxPWM = 255;
double wristKp = 4.0, wristKd = 0.5;
int lastWristError = 0;

// Serial Buffer
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Time tracking for PID loop frequency
unsigned long lastPIDTime = 0;

void setup() {
  Serial.begin(115200);
  
  int storedBase = EEPROM.read(ADDR_BASE);
  if (storedBase > 180) { // Invalid angle value
      storedBase = 90; // Default center if memory is empty
    }

  baseCurrent = storedBase;
  baseTarget = storedBase;
  baseServo.write(storedBase);

  // 1. Setup Servos
  baseServo.attach(BASE_PIN);
  // gripperServo.attach(GRIPPER_PIN);
  
  // 2. Setup DC Motors
  pinMode(ELBOW_PWM, OUTPUT);
  pinMode(ELBOW_IN1, OUTPUT);
  pinMode(ELBOW_IN2, OUTPUT);
  pinMode(ELBOW_POT, INPUT);
  
  pinMode(WRIST_PWM, OUTPUT);
  pinMode(WRIST_IN1, OUTPUT);
  pinMode(WRIST_IN2, OUTPUT);
  pinMode(WRIST_POT, INPUT);

  Serial.println("System Ready. Send: <ID, Angle, Speed(0-100)>");
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {    //if a command is received
    parseData();
    newData = false;
  }

  updateBase();
  // updateElbow();
  // updateWrist();
}

// --- Protocol: Read data like "<1, 180>" ---
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) { ndx = numChars - 1; }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// --- Logic: Decide what to do ---
void parseData() {
  char * strtokIndx; 
  
  // ID
  strtokIndx = strtok(receivedChars, ",");      
  int id = atoi(strtokIndx); 
  
  // Angle
  strtokIndx = strtok(NULL, ","); 
  int angle = atoi(strtokIndx);

  // Speed (0-100)
  strtokIndx = strtok(NULL, ",");
  int speedVal = atoi(strtokIndx); 

  switch(id) {
    case 1: // Base (Servo Ramping)
      baseTarget = angle;
      // Map 0-100 speed to Delay (100 = 0ms delay, 1 = 30ms delay)
      if(speedVal >= 100) baseStepDelay = 0;
      else baseStepDelay = map(speedVal, 1, 99, 30, 2);
      EEPROM.update(ADDR_BASE, angle); // Save position to EEPROM
      break;

    case 2: // Elbow (DC PID)
      elbowTarget = angle;
      // Map speed 0-100 to PWM limit 0-255
      elbowMaxPWM = map(speedVal, 0, 100, 0, 255);
      break;

    case 3: // Wrist (DC PID)
      wristTarget = angle;
      wristMaxPWM = map(speedVal, 0, 100, 0, 255);
      break;

    case 4: // Gripper (Direct Servo)
      // Speed is ignored for gripper
      gripperServo.write(angle);
      break;
  }
}

void updateBase() {
  // If instant speed, just write and exit
  if (baseStepDelay == 0) {
    if ((int)baseCurrent != baseTarget) {
      baseCurrent = baseTarget;
      baseServo.write((int)baseCurrent);
    }
    return;
  }

  // Timed step movement
  if (millis() - lastBaseTime >= baseStepDelay) {
    lastBaseTime = millis();
    if (baseCurrent < baseTarget) baseCurrent += 1.0;
    else if (baseCurrent > baseTarget) baseCurrent -= 1.0;
    baseServo.write((int)baseCurrent);
  }
}

void setMotor(int pwmPin, int in1, int in2, int speed, int potPin, int& lastError, int target, double kp, double kd, int maxPWM) {
  
  // Read Pot (0-1023) -> Convert to Degrees (0-180)
  // [IMPORTANT]: You must tweak these map values for your specific robot limits!
  int currentAngle = map(analogRead(potPin), 0, 1023, 0, 180);

  // Calculate PID
  int error = target - currentAngle;
  int motorSpeed = (error * kp) + ((error - lastError) * kd);
  lastError = error;

  // Clamp Speed
  int absSpeed = abs(motorSpeed);
  absSpeed = constrain(absSpeed, 0, maxPWM);

  // Deadzone (Prevent overheating if close enough)
  if (abs(error) < 3) absSpeed = 0; 

  // Output
  if (motorSpeed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, absSpeed);
}

void updateElbow() {
  if (millis() - lastPIDTime < 10) return; // Run at ~100Hz
  // No need to reset timer here, we use one timer for all PIDs to sync them
  
  setMotor(ELBOW_PWM, ELBOW_IN1, ELBOW_IN2, elbowMaxPWM, ELBOW_POT, lastElbowError, elbowTarget, elbowKp, elbowKd, elbowMaxPWM);
}

void updateWrist() {
  if (millis() - lastPIDTime < 10) return; 
  lastPIDTime = millis(); // Reset timer once per cycle
  
  setMotor(WRIST_PWM, WRIST_IN1, WRIST_IN2, wristMaxPWM, WRIST_POT, lastWristError, wristTarget, wristKp, wristKd, wristMaxPWM);
}