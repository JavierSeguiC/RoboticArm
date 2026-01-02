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
const int ADDR_GRIPPER = 1;

// Safety Limits
int baseMin = 0, baseMax = 180;
int elbowMin = 0, elbowMax = 180;
int wristMin = 0, wristMax = 180;
int gripperMin = 0, gripperMax = 180;

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

// Gripper Servo (Ramped Speed Logic)
int gripperTarget = 90;
float gripperCurrent = 90.0;
int gripperStepDelay = 0; 
unsigned long lastGripperTime = 0;

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

// --- SENSOR CALIBRATION ---
// For now, we assume full range 0-1023 corresponds to 0-180 degrees.
const int ELBOW_POT_MIN = 0;   // Analog value at 0 degrees
const int ELBOW_POT_MAX = 1023; // Analog value at 180 degrees

// --- SENSOR STATE ---
int elbowAngleCurrent = 0; // The calculated angle
int lastElbowRaw = -1;     // To track changes and prevent print flooding
const int SENSOR_THRESHOLD = 3; // Sensitivity: Ignore noise changes smaller than +/- 3

// Serial Buffer
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Time tracking for PID loop frequency
unsigned long lastPIDTime = 0;

void setup() {
  Serial.begin(115200);
  
  // Base servo setup
  int storedBase = EEPROM.read(ADDR_BASE);
  if (storedBase > 180) { // Invalid angle value
      storedBase = 90; // Default center if memory is empty
    }

  baseCurrent = storedBase;
  baseTarget = storedBase;
  baseServo.write(storedBase);
  baseServo.attach(BASE_PIN);

  int storedGripper = EEPROM.read(ADDR_GRIPPER);
  if (storedGripper > 180) storedGripper = 90; 

  gripperCurrent = storedGripper; 
  gripperTarget = storedGripper;
  gripperServo.write(storedGripper);
  gripperServo.attach(GRIPPER_PIN);
  
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

  readSensors();

  updateBase();
  updateGripper();
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
  strtokIndx = strtok(receivedChars, ","); int id = atoi(strtokIndx); 
  strtokIndx = strtok(NULL, ","); int val1 = atoi(strtokIndx); // Angle OR Min
  strtokIndx = strtok(NULL, ","); int val2 = atoi(strtokIndx); // Speed OR Max

  switch(id) {
    // --- MOVE COMMANDS ---
    case 1: // Base Move
      baseTarget = constrain(val1, baseMin, baseMax); // Safety Clamp
      if(val2 >= 100) baseStepDelay = 0; else baseStepDelay = map(val2, 1, 99, 30, 2);
      EEPROM.update(ADDR_BASE, baseTarget); 
      break;

    case 2: // Elbow Move
      elbowTarget = constrain(val1, elbowMin, elbowMax); // Safety Clamp
      elbowMaxPWM = map(val2, 0, 100, 0, 255);
      break;

    case 3: // Wrist Move
      wristTarget = constrain(val1, wristMin, wristMax); // Safety Clamp
      wristMaxPWM = map(val2, 0, 100, 0, 255);
      break;

    case 4: // Gripper Move
      gripperTarget = constrain(val1, gripperMin, gripperMax); // Safety Clamp
      if(val2 >= 100) gripperStepDelay = 0; else gripperStepDelay = map(val2, 1, 99, 30, 2);
      EEPROM.update(ADDR_GRIPPER, gripperTarget); 
      break;

    // --- CONFIGURATION COMMANDS (Set Limits) ---
    // Format: <11, Min, Max>
    case 11: baseMin = val1; baseMax = val2; Serial.println("Base Limits Updated"); break;
    case 12: elbowMin = val1; elbowMax = val2; Serial.println("Elbow Limits Updated"); break;
    case 13: wristMin = val1; wristMax = val2; Serial.println("Wrist Limits Updated"); break;
    case 14: gripperMin = val1; gripperMax = val2; Serial.println("Gripper Limits Updated"); break;
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

void updateGripper() {
  if (gripperStepDelay == 0) {
    if ((int)gripperCurrent != gripperTarget) {
      gripperCurrent = gripperTarget;
      gripperServo.write((int)gripperCurrent);
    }
    return;
  }

  if (millis() - lastGripperTime >= gripperStepDelay) {
    lastGripperTime = millis();
    if (gripperCurrent < gripperTarget) gripperCurrent += 1.0;
    else if (gripperCurrent > gripperTarget) gripperCurrent -= 1.0;
    gripperServo.write((int)gripperCurrent);
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

void readSensors() {
  // 1. Read Raw Value
  int rawValue = analogRead(ELBOW_POT);

  // 2. Filter Noise & Check for Change
  // We only update if the change is significant ( > threshold) to stop jitter
  if (abs(rawValue - lastElbowRaw) > SENSOR_THRESHOLD) {
    
    // 3. Map Raw Analog (0-1023) to Degrees (0-180)
    // If your pot is reversed, swap the last two numbers: map(..., 180, 0)
    elbowAngleCurrent = map(rawValue, ELBOW_POT_MIN, ELBOW_POT_MAX, 0, 180);
    
    // 4. Debug Print
    Serial.print("DEBUG >> Elbow Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Angle: ");
    Serial.println(elbowAngleCurrent);

    // Update last known value
    lastElbowRaw = rawValue;
  }
}