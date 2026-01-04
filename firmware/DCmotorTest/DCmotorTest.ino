// --- PIN DEFINITIONS ---
// Must match your wiring!
const int ENA = 3;  // PWM Speed Pin (Must be a ~ pin)
const int IN1 = 4;  // Direction Pin 1
const int IN2 = 5;  // Direction Pin 2

void setup() {
  // Set all pins as Outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Motor Test Start");
}

void loop() {
  // --- TEST 1: Forward at Full Speed ---
  Serial.println("Forward - Fast");
  
  // Direction Logic: HIGH/LOW = Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Speed Logic: 255 = 100% Duty Cycle (Max Speed)
  analogWrite(ENA, 255); 
  
  delay(2000); // Run for 2 seconds

  // --- TEST 2: Stop ---
  Serial.println("Stop");
  
  // Cut power to the motor
  analogWrite(ENA, 0); 
  // Optional: Brake by setting both IN pins LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  delay(1000);

  // --- TEST 3: Backward at Low Speed ---
  Serial.println("Backward - Slow");
  
  // Direction Logic: LOW/HIGH = Backward (Reverse of Test 1)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Speed Logic: 100 = ~40% Speed
  analogWrite(ENA, 100); 
  
  delay(2000);

  // --- TEST 4: Stop ---
  Serial.println("Stop");
  analogWrite(ENA, 0);
  delay(1000);
}