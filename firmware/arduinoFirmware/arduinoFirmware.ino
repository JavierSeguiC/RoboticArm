#include <Servo.h>
#include <EEPROM.h>

// --- Configuration ---
Servo baseServo;
Servo gripperServo;

// --- Pins ---
const int BASE_PIN = 9;
const int GRIPPER_PIN = 6;

// --- EEPROM Addresses ---
const int ADDR_BASE = 0;
const int ADDR_GRIPPER = 1;

// --- Safety Limits ---
int baseMin = 0, baseMax = 180;
int elbowMin = 0, elbowMax = 180;
int wristMin = 0, wristMax = 180;
int gripperMin = 0, gripperMax = 180;

// --- DC Motor 1: ELBOW ---
const int ELBOW_PWM = 11;  // Speed pin
const int ELBOW_IN1 = 7;  // Direction 1
const int ELBOW_IN2 = 8;  // Direction 2
const int ELBOW_POT = A1; // Potentiometer

// --- DC Motor 2: WRIST ---
const int WRIST_PWM = 3;
const int WRIST_IN1 = 4; 
const int WRIST_IN2 = 5;
const int WRIST_POT = A0;

// --- Base Servo (Ramped Speed Logic) ---
int baseTarget = 90;
float baseCurrent = 90.0;
int baseStepDelay = 0; // Delay in ms (0 = Fast, Higher = Slower)
unsigned long lastBaseTime = 0;

// --- Gripper Servo (Ramped Speed Logic)
int gripperTarget = 90;
float gripperCurrent = 90.0;
int gripperStepDelay = 0; 
unsigned long lastGripperTime = 0;

// --- PID Constants  ---
double wristKp = 6.0, wristKd = 2.0; 
double elbowKp = 6.0, elbowKd = 2.0;

// --- Elbow DC (PID Logic) ---
int elbowTarget = 90;
int elbowMaxPWM = 255; // Speed limit
int lastElbowError = 0;
unsigned long lastElbowTime = 0; // For timing PID updates

// --- Wrist DC (PID Logic) ---
int wristTarget = 90;
int wristMaxPWM = 255;
int lastWristError = 0;
unsigned long lastWristTime = 0; // For timing PID updates

// --- Sensor Callibration ---
// For now, we assume full range 0-1023 corresponds to 0-180 degrees.
const int POT_MIN = 0;   // Analog value at 0 degrees
const int POT_MAX = 1023; // Analog value at 180 degrees

// --- Sensor State ---
int elbowAngleCurrent = 0; // The calculated angle
int lastElbowRaw = -1;     // To track changes and prevent print flooding

int wristAngleCurrent = 0;
int lastWristRaw = -1;  

const int SENSOR_THRESHOLD = 3; // Sensitivity: Ignore noise changes smaller than +/- 3

// --- Reporting ---
unsigned long lastReportTime = 0;

// Serial Buffer
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

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

  // Gripper servo setup
  int storedGripper = EEPROM.read(ADDR_GRIPPER);
  if (storedGripper > 180) storedGripper = 90; 

  gripperCurrent = storedGripper; 
  gripperTarget = storedGripper;
  gripperServo.write(storedGripper);
  gripperServo.attach(GRIPPER_PIN);
  
  // Wrist DC Motor setup
  pinMode(WRIST_PWM, OUTPUT); 
  pinMode(WRIST_IN1, OUTPUT); 
  pinMode(WRIST_IN2, OUTPUT); 
  pinMode(WRIST_POT, INPUT);

  int currentWristAngle = map(analogRead(WRIST_POT), 0, 1023, 0, 180);
  wristTarget = currentWristAngle; // Hold current position

  // Elbow DC Motor setup
  pinMode(ELBOW_PWM, OUTPUT); 
  pinMode(ELBOW_IN1, OUTPUT); 
  pinMode(ELBOW_IN2, OUTPUT); 
  pinMode(ELBOW_POT, INPUT);

  // int currentElbowAngle = map(analogRead(ELBOW_POT), 0, 1023, 0, 180);
  // elbowTarget = currentElbowAngle;

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
  updateElbow();
  updateWrist();

  reportTelemetry();
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

// --- GENERIC PID CONTROL FUNCTION ---
void setMotor(int pwmPin, int in1, int in2, int potPin, int& lastError, int target, double kp, double kd, int maxPWM) {
  
  // 1. Read Sensor
  int currentAngle = map(analogRead(potPin), 0, 1023, 180, 0);

  /* // --- DEBUG PRINT (Uncomment to see values) ---
  Serial.print("DEBUG >> Curr: "); Serial.print(pwmPin); Serial.print(currentAngle);
  Serial.print(" | Targ: "); Serial.println(target); */

  // 2. Calculate PID
  int error = target - currentAngle;
  int motorSpeed = (error * kp) + ((error - lastError) * kd);
  lastError = error;

 /*  // --- DEBUG PRINT (Uncomment to see values) ---
  Serial.print("DEBUG >> Error: "); Serial.print(pwmPin); Serial.print(error);
  Serial.print(" | Speed (Pre-Limit): "); Serial.println(motorSpeed); */

  // 3. Deadzone & Max Speed
  if (abs(error) <= 2) {
    motorSpeed = 0;
  }
  
  int absSpeed = abs(motorSpeed);
  absSpeed = constrain(absSpeed, 0, maxPWM);

  // 4. Drive Motor
  if (motorSpeed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  
  analogWrite(pwmPin, absSpeed);
}

void updateWrist() {
  if (millis() - lastWristTime < 10) return; 
  lastWristTime = millis(); 
  
  setMotor(WRIST_PWM, WRIST_IN1, WRIST_IN2, WRIST_POT, lastWristError, wristTarget, wristKp, wristKd, wristMaxPWM);
}

void updateElbow() {
  // Use specific elbow timer
  if (millis() - lastElbowTime < 10) return; 
  lastElbowTime = millis();

  setMotor(ELBOW_PWM, ELBOW_IN1, ELBOW_IN2, ELBOW_POT, lastElbowError, elbowTarget, elbowKp, elbowKd, elbowMaxPWM);
}

void readSensors() {
  // 1. Read Raw Value
  int rawElbow = analogRead(ELBOW_POT);
  int rawWrist = analogRead(WRIST_POT);

  // 2. Filter Noise & Check for Change
  // We only update if the change is significant ( > threshold) to stop jitter
  if (abs(rawElbow - lastElbowRaw) > SENSOR_THRESHOLD) {
    
    // 3. Map Raw Analog (0-1023) to Degrees (0-180)
    // If your pot is reversed, swap the last two numbers: map(..., 180, 0)
    elbowAngleCurrent = map(rawElbow, POT_MIN, POT_MAX, 0, 180);
    
    // Update last known value
    lastElbowRaw = rawElbow;
  }

  if (abs(rawWrist - lastWristRaw) > SENSOR_THRESHOLD) {
    wristAngleCurrent = map(rawWrist, POT_MIN, POT_MAX, 0, 180);
    lastWristRaw = rawWrist;
  }
}

void reportTelemetry() {
  // Report every 200ms (5Hz) to avoid flooding the serial line
  if (millis() - lastReportTime < 200) return;
  lastReportTime = millis();

  // Format: STATUS:ElbowAngle,ElbowRaw,WristAngle,WristRaw
  // We use a simplified format to make parsing easier in Python
  Serial.print("STATUS:");
  Serial.print(elbowAngleCurrent);
  Serial.print(",");
  Serial.print(lastElbowRaw);
  Serial.print(",");
  Serial.print(wristAngleCurrent);
  Serial.print(",");
  Serial.println(lastWristRaw);
}