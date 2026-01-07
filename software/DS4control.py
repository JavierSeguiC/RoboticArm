import pygame
import serial
import time
import sys

# --- CONFIGURATION ---
SERIAL_PORT = 'COM8'   # REPLACE with your Arduino Port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD_RATE = 115200
LOOP_RATE = 20         # How many times per second to update the robot (Hz)

# Sensitivity (How fast the joint moves when stick is full throttle)
SPEED_BASE = 2.0       # Degrees per loop
SPEED_ELBOW = 2.0
SPEED_WRIST = 2.0
SPEED_GRIPPER = 3.0

# --- INITIAL ROBOT STATE ---
# We track the "Ghost Target" here in Python
current_base = 90.0
current_elbow = 90.0
current_wrist = 90.0
current_gripper = 90.0

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Wait for Arduino reset
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"ERROR: Could not connect to {SERIAL_PORT}. Is it plugged in? Is Serial Monitor closed?")
    sys.exit()

# --- SETUP PYGAME (CONTROLLER) ---
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No Controller Found! Please connect DualShock 4 via Bluetooth/USB.")
    sys.exit()

# Initialize the first controller
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller Connected: {joystick.get_name()}")

# --- HELPER FUNCTIONS ---
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def send_update(id, angle):
    # Send strictly formatted command: <ID, Angle, Speed>
    # We use a high 'Speed' value (80-100) because Python is controlling the ramping now.
    cmd = f"<{id},{int(angle)},80>" 
    ser.write(cmd.encode())

# --- MAIN LOOP ---
try:
    clock = pygame.time.Clock()
    
    while True:
        # 1. Handle Window Events (Required by Pygame)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        # 2. READ JOYSTICK AXES
        # Note: DS4 Axis mappings vary by OS. These are standard Windows mappings.
        # Axis 0: Left Stick X (Base)
        # Axis 1: Left Stick Y (Elbow)
        # Axis 2: Right Stick X (Unused or Wrist)
        # Axis 3: Right Stick Y (Wrist)
        # Axis 4/5: Triggers (Gripper) - sometimes mapped as Buttons
        
        raw_base_x = joystick.get_axis(0)  
        raw_elbow_y = joystick.get_axis(1) 
        raw_wrist_y = joystick.get_axis(3) 

        # Read Triggers (L2/R2 often range -1.0 to 1.0)
        # We combine them: R2 (Close) - L2 (Open)
        # Note: On some drivers triggers are Axis 4 and 5. Check the print output!
        raw_l2 = joystick.get_axis(4) 
        raw_r2 = joystick.get_axis(5)
        
        # Deadzone Filter (Ignore tiny drift)
        def deadzone(val):
            return 0.0 if abs(val) < 0.1 else val

        val_base = deadzone(raw_base_x)
        val_elbow = deadzone(raw_elbow_y) * -1 # Invert Y axis (Up is usually -1)
        val_wrist = deadzone(raw_wrist_y) * -1
        
        # Calculate Gripper Input (R2 is usually -1 when released, 1 when pressed)
        # Let's normalize them to 0.0 -> 1.0
        grip_close = (raw_r2 + 1) / 2.0
        grip_open  = (raw_l2 + 1) / 2.0
        val_gripper = grip_close - grip_open # Positive = Close, Negative = Open

        # --- 3. PRINT DEBUG INFO (As requested) ---
        # \r allows overwriting the same line for a clean terminal
        print(f"\r[INPUT] B:{val_base:.2f} E:{val_elbow:.2f} W:{val_wrist:.2f} G:{val_gripper:.2f}  |  [TGT] B:{int(current_base)} E:{int(current_elbow)} W:{int(current_wrist)}", end="")

        # --- 4. CALCULATE NEW TARGETS (The "Integrator") ---
        # This is where the "Gas Pedal" magic happens.
        # Current = Current + (Direction * Speed)
        
        current_base += val_base * SPEED_BASE
        current_elbow += val_elbow * SPEED_ELBOW
        current_wrist += val_wrist * SPEED_WRIST
        current_gripper += val_gripper * SPEED_GRIPPER

        # Constrain to physical limits (0-180)
        current_base = max(0, min(180, current_base))
        current_elbow = max(0, min(180, current_elbow))
        current_wrist = max(0, min(180, current_wrist))
        current_gripper = max(0, min(180, current_gripper))

        # --- 5. SEND TO ARDUINO ---
        # We send updates for all joints every loop. 
        # Since Python runs at 20Hz, this is smooth.
        send_update(1, current_base)
        send_update(2, current_elbow)
        send_update(3, current_wrist)
        send_update(4, current_gripper)

        # Loop timing control
        clock.tick(LOOP_RATE)

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()