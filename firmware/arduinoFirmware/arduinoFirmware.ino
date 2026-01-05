#include <Servo.h>
#include <EEPROM.h>

// --- Configuration ---
Servo baseServo;
Servo gripperServo;

// --- Pins ---
const int BASE_PIN = 9;
const int GRIPPER_PIN = 6;

// --- Variables for DC speed control ---

int currentElbowPWM = 0;
int currentWristPWM = 0;

// Current "Ramped" Setpoints (Float for smooth movement)
float elbowCurrentSetpoint = 90.0;
float wristCurrentSetpoint = 90.0;

// Speed Steps (Degrees per loop update)
float elbowStepSize = 5.0; 
float wristStepSize = 5.0;

// Minimum PWM to overcome friction (50-70)
const int MIN_PWM = 60;

// --- EEPROM MEMORY MAP ---
// 0-9: Last Known Positions
const int ADDR_POS_BASE    = 0;
const int ADDR_POS_ELBOW   = 1;
const int ADDR_POS_WRIST   = 2;
const int ADDR_POS_GRIPPER = 3;

// 10-19: Safety Limits (Min/Max)
const int ADDR_LIM_BASE_MIN = 10;
const int ADDR_LIM_BASE_MAX = 11;
const int ADDR_LIM_ELBOW_MIN = 12;
const int ADDR_LIM_ELBOW_MAX = 13;
const int ADDR_LIM_WRIST_MIN = 14;
const int ADDR_LIM_WRIST_MAX = 15;
const int ADDR_LIM_GRIP_MIN  = 16;
const int ADDR_LIM_GRIP_MAX  = 17;

// 20-29: Saved Configurations (e.g., HOME)
const int ADDR_HOME_BASE    = 20;
const int ADDR_HOME_ELBOW   = 21;
const int ADDR_HOME_WRIST   = 22;
const int ADDR_HOME_GRIPPER = 23;

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
double wristKp = 12.0, wristKd = 6.0; 
double elbowKp = 12.0, elbowKd = 6.0;

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

  TCCR2B = (TCCR2B & 0b11111000) | 0x01; // Set PWM frequency to 31.4kHz for pins 3, 9, 10, 11
                                         // outside of human hearing range
  
  loadLimits();

  baseTarget = readEEPROM(ADDR_POS_BASE, 90);
  elbowTarget = readEEPROM(ADDR_POS_ELBOW, 90);
  wristTarget = readEEPROM(ADDR_POS_WRIST, 90);
  gripperTarget = readEEPROM(ADDR_POS_GRIPPER, 90);

  baseCurrent = baseTarget; 
  baseServo.write(baseCurrent);
  baseServo.attach(BASE_PIN);
  
  gripperCurrent = gripperTarget;
  gripperServo.write(gripperCurrent);
  gripperServo.attach(GRIPPER_PIN);
  
  // Wrist DC Motor setup
  wristCurrentSetpoint = wristTarget;
  pinMode(WRIST_PWM, OUTPUT); 
  pinMode(WRIST_IN1, OUTPUT); 
  pinMode(WRIST_IN2, OUTPUT); 
  pinMode(WRIST_POT, INPUT);

  // Elbow DC Motor setup
  elbowCurrentSetpoint = elbowTarget; 
  pinMode(ELBOW_PWM, OUTPUT); 
  pinMode(ELBOW_IN1, OUTPUT); 
  pinMode(ELBOW_IN2, OUTPUT); 
  pinMode(ELBOW_POT, INPUT);

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

int readEEPROM(int addr, int def) {
  int val = EEPROM.read(addr);
  if (val == 255) return def; // 255 usually means empty EEPROM
  return val;
}

void loadLimits() {
  baseMin = readEEPROM(ADDR_LIM_BASE_MIN, 0);
  baseMax = readEEPROM(ADDR_LIM_BASE_MAX, 180);
  
  elbowMin = readEEPROM(ADDR_LIM_ELBOW_MIN, 0);
  elbowMax = readEEPROM(ADDR_LIM_ELBOW_MAX, 180);
  
  wristMin = readEEPROM(ADDR_LIM_WRIST_MIN, 0);
  wristMax = readEEPROM(ADDR_LIM_WRIST_MAX, 180);
  
  gripperMin = readEEPROM(ADDR_LIM_GRIP_MIN, 0);
  gripperMax = readEEPROM(ADDR_LIM_GRIP_MAX, 180);
}

void saveLimits() {
  EEPROM.update(ADDR_LIM_BASE_MIN, baseMin); EEPROM.update(ADDR_LIM_BASE_MAX, baseMax);
  EEPROM.update(ADDR_LIM_ELBOW_MIN, elbowMin); EEPROM.update(ADDR_LIM_ELBOW_MAX, elbowMax);
  EEPROM.update(ADDR_LIM_WRIST_MIN, wristMin); EEPROM.update(ADDR_LIM_WRIST_MAX, wristMax);
  EEPROM.update(ADDR_LIM_GRIP_MIN, gripperMin); EEPROM.update(ADDR_LIM_GRIP_MAX, gripperMax);
}

void saveHomePosition() {
  EEPROM.update(ADDR_HOME_BASE, baseTarget);
  EEPROM.update(ADDR_HOME_ELBOW, elbowTarget);
  EEPROM.update(ADDR_HOME_WRIST, wristTarget);
  EEPROM.update(ADDR_HOME_GRIPPER, gripperTarget);
  Serial.println("Home Position Saved.");
}

void loadHomePosition() {
  baseTarget = readEEPROM(ADDR_HOME_BASE, 90);
  elbowTarget = readEEPROM(ADDR_HOME_ELBOW, 90);
  wristTarget = readEEPROM(ADDR_HOME_WRIST, 90);
  gripperTarget = readEEPROM(ADDR_HOME_GRIPPER, 90);
  
  // Also update Last Pos immediately so if we reset, we stay here
  EEPROM.update(ADDR_POS_BASE, baseTarget);
  EEPROM.update(ADDR_POS_ELBOW, elbowTarget);
  EEPROM.update(ADDR_POS_WRIST, wristTarget);
  EEPROM.update(ADDR_POS_GRIPPER, gripperTarget);
  
  Serial.println("Moving to Home...");
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
      EEPROM.update(ADDR_POS_BASE, baseTarget); 
      break;

    case 2: // Elbow Move
      elbowTarget = constrain(val1, elbowMin, elbowMax); // Safety Clamp
      if (val2 >= 100) elbowStepSize = 100.0; // Instant
      else elbowStepSize = map(val2, 1, 99, 10, 100) / 10.0; 
      break;

    case 3: // Wrist Move
      wristTarget = constrain(val1, wristMin, wristMax); // Safety Clamp
      if (val2 >= 100) wristStepSize = 100.0;
      else wristStepSize = map(val2, 1, 99, 10, 100) / 10.0;
      break;

    case 4: // Gripper Move
      gripperTarget = constrain(val1, gripperMin, gripperMax); // Safety Clamp
      if(val2 >= 100) gripperStepDelay = 0; else gripperStepDelay = map(val2, 1, 99, 30, 2);
      EEPROM.update(ADDR_POS_GRIPPER, gripperTarget); 
      break;

    // --- CONFIGURATION COMMANDS (Set Limits) ---
    case 11: baseMin = val1; baseMax = val2; Serial.println("Base Limits Updated"); break;
    case 12: elbowMin = val1; elbowMax = val2; Serial.println("Elbow Limits Updated"); break;
    case 13: wristMin = val1; wristMax = val2; Serial.println("Wrist Limits Updated"); break;
    case 14: gripperMin = val1; gripperMax = val2; Serial.println("Gripper Limits Updated"); break;
    
    // --- HOME COMMANDS ---
    case 98: loadHomePosition(); break; // Go Home
    case 99: saveHomePosition(); break; // Set Home
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
int setMotor(int pwmPin, int in1, int in2, int potPin, int& lastError, int target, double kp, double kd, int maxPWM) {
  
  // 1. Read Sensor
  int currentAngle = map(analogRead(potPin), 0, 1023, 180, 0);

  // 2. Calculate PID
  int error = target - currentAngle;
  int motorSpeed = (error * kp) + ((error - lastError) * kd);
  lastError = error;

  // 3. Deadzone & Max Speed
  if (abs(error) <= 1) { 
    motorSpeed = 0;
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    return; // Exit early
  }
  
  if (motorSpeed > 0 && motorSpeed < MIN_PWM) motorSpeed = MIN_PWM;
  if (motorSpeed < 0 && motorSpeed > -MIN_PWM) motorSpeed = -MIN_PWM;

  int absSpeed = constrain(abs(motorSpeed), 0, maxPWM);

  if (motorSpeed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  }
  
  analogWrite(pwmPin, absSpeed);

  return absSpeed;
}

void updateWrist() {
  if (millis() - lastWristTime < 20) return; 
  lastWristTime = millis(); 
  
  // --- 1. RAMPING LOGIC ---
  if (abs(wristCurrentSetpoint - wristTarget) > wristStepSize) {
    if (wristCurrentSetpoint < wristTarget) wristCurrentSetpoint += wristStepSize;
    else wristCurrentSetpoint -= wristStepSize;
  } else {
    wristCurrentSetpoint = wristTarget; // Snap to finish
  }

  // --- 2. CALL PID with RAMPED Setpoint and FULL POWER (255) ---
  // Note: We cast setpoint to (int) for the PID calculation
  currentWristPWM = setMotor(WRIST_PWM, WRIST_IN1, WRIST_IN2, WRIST_POT, lastWristError, (int)wristCurrentSetpoint, wristKp, wristKd, 255);
}

void updateElbow() {
  // Use specific elbow timer
  if (millis() - lastElbowTime < 20) return; 
  lastElbowTime = millis();

  // --- 1. RAMPING LOGIC ---
  if (abs(elbowCurrentSetpoint - elbowTarget) > elbowStepSize) {
    if (elbowCurrentSetpoint < elbowTarget) elbowCurrentSetpoint += elbowStepSize;
    else elbowCurrentSetpoint -= elbowStepSize;
  } else {
    elbowCurrentSetpoint = elbowTarget; // Snap to finish
  }

  // --- 2. CALL PID with RAMPED Setpoint and FULL POWER (255) ---
  // Note: We cast setpoint to (int) for the PID calculation
  currentElbowPWM = setMotor(ELBOW_PWM, ELBOW_IN1, ELBOW_IN2, ELBOW_POT, lastElbowError, (int)elbowCurrentSetpoint, elbowKp, elbowKd, 255);
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
  Serial.print(elbowAngleCurrent); // Elbow measured angle (º)
  Serial.print(",");
  Serial.print(lastElbowRaw);      // Elbow raw sensor value
  Serial.print(",");
  Serial.print(wristAngleCurrent); // Wrist measured angle (º)
  Serial.print(",");
  Serial.print(lastWristRaw);      // Wrist raw sensor value
  Serial.print(",");    
  Serial.print(map(currentElbowPWM, 0, 255, 0, 100)); Serial.print(","); // Elbow PWM as %
  Serial.println(map(currentWristPWM, 0, 255, 0, 100));               // Wrist PWM as %
}