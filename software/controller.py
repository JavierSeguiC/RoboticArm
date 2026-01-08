import tkinter as tk
from tkinter import ttk
import serial
import time
import threading
import pygame

# --- CONFIGURATION ---
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

class RobotControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Manipulator Control")
        self.root.geometry("500x750")
        
        self.speed_val = 50 #default speed
        self.ser = None
        self.running = True

        self.target_base = 90
        self.target_elbow = 90
        self.target_wrist = 90
        self.target_gripper = 90

        self.telemetry_data = {
            "base": 90, "elbow": 90, "wrist": 90, "gripper": 90
        }
        self.startup_synced = False  # <--- ADD THIS FLAG

        self.remote_active = False
        self.joy_sensitivity = [2.0, 2.0, 2.0, 3.0] # Speed for Base, Elbow, Wrist, Gripper
        self.sliders = {} # We need to store sliders to move them programmatically later
        
        # [NEW] Setup Joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Controller Connected: {self.joystick.get_name()}")
        else:
            self.joystick = None
            print("No Controller Found")

        # [NEW] Start the Remote Polling Loop
        self.root.after(50, self.process_remote_input)
        
        self.create_widgets()
        self.connect_serial()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Wait for Arduino to reset
            print(f"Connected to {SERIAL_PORT}")

            self.ser.write(b"<97,0,0>")  # Request Status Update on Connect

            # This runs in the background and prints whatever Arduino says
            self.read_thread = threading.Thread(target=self.read_serial_data)
            self.read_thread.daemon = True # Kills thread when main app closes
            self.read_thread.start()

        except Exception as e:
            print(f"Error connecting: {e}")

    def read_serial_data(self):
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        if line.startswith("STATUS:"):
                            # Format: Base, Elbow, Wrist, Gripper, RawElbow, RawWrist, PWM_Elbow, PWM_Wrist
                            parts = line.split(":")[1].split(",")
                            
                            if len(parts) >= 8:
                                # 1. Parse Data
                                self.telemetry_data["base"] = int(parts[0])
                                
                                # [FIX] Invert DC Motors (180 - Angle) to match visual polarity
                                self.telemetry_data["elbow"] = 180 - int(parts[1])
                                self.telemetry_data["wrist"] = 180 - int(parts[2])
                                
                                self.telemetry_data["gripper"] = int(parts[3])
                                
                                # Store raw data for debugging
                                self.telemetry_data["elbow_raw"] = parts[4]
                                self.telemetry_data["wrist_raw"] = parts[5]
                                self.telemetry_data["elbow_pwm"] = parts[6]
                                self.telemetry_data["wrist_pwm"] = parts[7]

                                # 2. Sync Sliders on Startup
                                if not self.startup_synced:
                                    self.sync_sliders_to_telemetry()
                                    self.startup_synced = True
                                    print("System Synced with Robot State")
                                    
                                self.update_telemetry_ui()

                        elif line.startswith("CONFIG:"):
                            # Format: minB, maxB, minE, maxE, minW, maxW, minG, maxG
                            parts = line.split(":")[1].split(",")
                            if len(parts) >= 8:
                                # Helper to update GUI safely from thread
                                def update_entries():
                                    # Base
                                    self.limit_entries[11]["min"].delete(0, tk.END); self.limit_entries[11]["min"].insert(0, parts[0])
                                    self.limit_entries[11]["max"].delete(0, tk.END); self.limit_entries[11]["max"].insert(0, parts[1])
                                    # Elbow
                                    self.limit_entries[12]["min"].delete(0, tk.END); self.limit_entries[12]["min"].insert(0, parts[2])
                                    self.limit_entries[12]["max"].delete(0, tk.END); self.limit_entries[12]["max"].insert(0, parts[3])
                                    # Wrist
                                    self.limit_entries[13]["min"].delete(0, tk.END); self.limit_entries[13]["min"].insert(0, parts[4])
                                    self.limit_entries[13]["max"].delete(0, tk.END); self.limit_entries[13]["max"].insert(0, parts[5])
                                    # Gripper
                                    self.limit_entries[14]["min"].delete(0, tk.END); self.limit_entries[14]["min"].insert(0, parts[6])
                                    self.limit_entries[14]["max"].delete(0, tk.END); self.limit_entries[14]["max"].insert(0, parts[7])
                                    
                                    print("UI Updated with stored limits from EEPROM")

                                self.root.after(0, update_entries)  

                        elif line:
                            print(f"[ARDUINO]: {line}") 
                except Exception as e:
                    print(f"Serial Read Error: {e}")
            time.sleep(0.01)

    def send_command(self, motor_id, angle, override_speed=None):
        # Update local target memory
        if motor_id == 1: self.target_base = angle
        elif motor_id == 2: self.target_elbow = angle
        elif motor_id == 3: self.target_wrist = angle
        elif motor_id == 4: self.target_gripper = angle
        
        speed_to_send = override_speed if override_speed is not None else self.speed_val

        if self.ser and self.ser.is_open:
            try:
                command = f"<{motor_id},{int(angle)},{speed_to_send}>"
                self.ser.write(command.encode())
                self.update_telemetry_ui()
                
            except serial.SerialTimeoutException:
                print("Warning: Serial Timeout - Arduino busy or resetting?")
                # Optional: Re-open the port if this happens too often
                
            except serial.SerialException as e:
                print(f"Serial Error: {e}")
                # If connection is totally lost, maybe stop the remote
                self.remote_active = False
                self.btn_remote.config(text="Enable Remote: OFF (Error)", bg="red")

    def send_limit_config(self, config_id, min_val, max_val):
        """Configuration Command: <ConfigID, Min, Max>"""
        if self.ser and self.ser.is_open:
            command = f"<{config_id},{min_val},{max_val}>"
            self.ser.write(command.encode())
            print(f"Config: {command}")

    def update_speed(self, val):
        self.speed_val = int(float(val))
        print(f"Speed set to: {self.speed_val}%")

    def create_widgets(self):
        # --- 1. TELEMETRY SECTION (Expandable) ---
        self.show_telemetry = tk.BooleanVar(value=True)
        tel_btn = ttk.Checkbutton(self.root, text="Show Real-Time Telemetry", 
                                  variable=self.show_telemetry, command=self.toggle_telemetry)
        tel_btn.pack(pady=5)

        self.telemetry_frame = ttk.LabelFrame(self.root, text="Joint Status Monitor")
        self.telemetry_frame.pack(pady=5, padx=10, fill='x')
        
        # Grid Headers
        ttk.Label(self.telemetry_frame, text="Joint", font=("Arial", 9, "bold")).grid(row=0, column=0, padx=5)
        ttk.Label(self.telemetry_frame, text="Target (°)", font=("Arial", 9, "bold")).grid(row=0, column=1, padx=5)
        ttk.Label(self.telemetry_frame, text="Actual (°)", font=("Arial", 9, "bold")).grid(row=0, column=2, padx=5)
        ttk.Label(self.telemetry_frame, text="Raw", font=("Arial", 9, "bold")).grid(row=0, column=3, padx=5)
        ttk.Label(self.telemetry_frame, text="PWM (%)", font=("Arial", 9, "bold")).grid(row=0, column=4, padx=5)
        
        # Grid Data Labels (We keep references to update them later)
        self.tel_labels = {}
        joints = ["Base", "Elbow", "Wrist", "Gripper"]
        for i, joint in enumerate(joints):
            row = i + 1
            ttk.Label(self.telemetry_frame, text=joint + ":").grid(row=row, column=0, sticky="e")

            l_target = ttk.Label(self.telemetry_frame, text="--")
            l_target.grid(row=row, column=1)

            l_actual = ttk.Label(self.telemetry_frame, text="--")
            l_actual.grid(row=row, column=2)

            l_raw = ttk.Label(self.telemetry_frame, text="--")
            l_raw.grid(row=row, column=3)

            l_pwm = ttk.Label(self.telemetry_frame, text="--") 
            l_pwm.grid(row=row, column=4)
            
            self.tel_labels[joint] = {"t": l_target, "a": l_actual, "r": l_raw, "p": l_pwm}
        
        # --- 2. CONTROLS SECTION ---
        ttk.Label(self.root, text="Manual Control", font=("Arial", 16)).pack(pady=10)

        # [NEW] Remote Toggle
        self.btn_remote = tk.Button(self.root, text="Enable Remote: OFF", bg="lightgray", 
                                    command=self.toggle_remote)
        self.btn_remote.pack(pady=5)

        speed_frame = ttk.LabelFrame(self.root, text="Global Velocity Limit (%)")
        speed_frame.pack(pady=5, padx=20, fill='x')
        speed_scale = ttk.Scale(speed_frame, from_=1, to=100, orient='horizontal', command=self.update_speed)
        speed_scale.set(50) 
        speed_scale.pack(fill='x', padx=10, pady=5)

        self.add_slider("Base Rotation", 1, 0, 180)
        self.add_slider("Elbow Joint", 2, 0, 180)
        self.add_slider("Wrist Joint", 3, 0, 180)

        frame = ttk.Frame(self.root)
        frame.pack(pady=10)
        ttk.Button(frame, text="Open Gripper", command=lambda: self.send_command(4, 180)).pack(side=tk.LEFT, padx=5)
        ttk.Button(frame, text="Close Gripper", command=lambda: self.send_command(4, 90)).pack(side=tk.LEFT, padx=5)

        # HOME BUTTONS  
        # Save Current Position as Home
        ttk.Button(root, text="Set Home", command=lambda: app.ser.write(b"<99,0,0>")).pack()

        # Move Robot to Home
        ttk.Button(root, text="Go Home", command=lambda: app.ser.write(b"<98,0,0>")).pack()

        # --- 3. LIMITS SECTION (Expandable) ---
        self.show_limits = tk.BooleanVar(value=False)
        toggle_btn = ttk.Checkbutton(self.root, text="Show Safety Limits Configuration", 
                                     variable=self.show_limits, command=self.toggle_limits_menu)
        toggle_btn.pack(pady=10)

        self.limits_frame = ttk.LabelFrame(self.root, text="Joint Angle Constraints")
        self.limit_entries = {} 
        self.add_limit_row("Base (ID 1)", 11)
        self.add_limit_row("Elbow (ID 2)", 12)
        self.add_limit_row("Wrist (ID 3)", 13)
        self.add_limit_row("Gripper (ID 4)", 14)

    def toggle_remote(self):
        self.remote_active = not self.remote_active
        if self.remote_active:
            self.btn_remote.config(text="Enable Remote: ON", bg="lightgreen")
        
            self.sync_sliders_to_telemetry()
            
        else:
            self.btn_remote.config(text="Enable Remote: OFF", bg="lightgray")
    def process_remote_input(self):
        # Only run if active and joystick exists
        if self.remote_active and self.joystick:
            pygame.event.pump() 

            # 1. Read Axes (Using your DS4 logic)
            # Adjust axis numbers if needed based on your testing
            def deadzone(val): return 0.0 if abs(val) < 0.1 else val

            axis_base   = deadzone(self.joystick.get_axis(0)) * -1 
            axis_elbow  = deadzone(self.joystick.get_axis(1))      
            axis_wrist  = deadzone(self.joystick.get_axis(3)) 
            
            # Triggers logic
            val_l2 = (self.joystick.get_axis(4) + 1) / 2
            val_r2 = (self.joystick.get_axis(5) + 1) / 2
            axis_gripper = val_l2 - val_r2

            inputs = [axis_base, axis_elbow, axis_wrist, axis_gripper]
            current_targets = [self.target_base, self.target_elbow, self.target_wrist, self.target_gripper]
            
            any_movement = False

            # 2. Update Shared State ("The Agreement")
            for i in range(4):
                if inputs[i] != 0:
                    any_movement = True
                    # Update target
                    current_targets[i] += inputs[i] * self.joy_sensitivity[i]
                    # Constrain
                    current_targets[i] = max(0, min(180, current_targets[i]))
                    
                    # [CRITICAL] Update the Slider Visual
                    # This ensures the UI agrees with the Remote
                    if (i + 1) in self.sliders:
                        self.sliders[i + 1].set(int(current_targets[i]))

            # 3. Save State & Send Commands
            if any_movement:
                # We use override_speed=80 for snappy remote response
                self.send_command(1, current_targets[0], 80)
                self.send_command(2, current_targets[1], 80)
                self.send_command(3, current_targets[2], 80)
                self.send_command(4, current_targets[3], 80)

        # 4. Schedule next check in 50ms
        self.root.after(50, self.process_remote_input)

    def sync_sliders_to_telemetry(self):
        # Update Target Variables to match Reality
        self.target_base = self.telemetry_data["base"]
        self.target_elbow = self.telemetry_data["elbow"]
        self.target_wrist = self.telemetry_data["wrist"]
        self.target_gripper = self.telemetry_data["gripper"]

        # Update Visual Sliders (Using .set does not trigger the callback usually)
        if 1 in self.sliders: self.sliders[1].set(self.target_base)
        if 2 in self.sliders: self.sliders[2].set(self.target_elbow)
        if 3 in self.sliders: self.sliders[3].set(self.target_wrist)
        if 4 in self.sliders: self.sliders[4].set(self.target_gripper)
    
    def toggle_telemetry(self):
        if self.show_telemetry.get():
            self.telemetry_frame.pack(pady=5, padx=10, fill='x')
        else:
            self.telemetry_frame.pack_forget()

    def update_telemetry_ui(self):
        # This function is called from the background thread, so we must schedule the UI update
        # to run on the main thread using after()
        self.root.after(0, self._update_ui_safe)

    def _update_ui_safe(self):
        # --- 1. Update Target Labels (From local variables) ---
        self.tel_labels["Base"]["t"].config(text=str(self.target_base))
        self.tel_labels["Elbow"]["t"].config(text=str(self.target_elbow))
        self.tel_labels["Wrist"]["t"].config(text=str(self.target_wrist))
        self.tel_labels["Gripper"]["t"].config(text=str(self.target_gripper))

        # --- 2. Update Actual/Raw/PWM Labels (From Telemetry) ---
        
        # Base (Servo)
        self.tel_labels["Base"]["a"].config(text=str(self.telemetry_data.get("base", "--")))
        self.tel_labels["Base"]["r"].config(text="N/A")
        self.tel_labels["Base"]["p"].config(text="N/A")
        
        # Elbow (DC Motor)
        self.tel_labels["Elbow"]["a"].config(text=str(self.telemetry_data.get("elbow", "--")))
        self.tel_labels["Elbow"]["r"].config(text=str(self.telemetry_data.get("elbow_raw", "--")))
        self.tel_labels["Elbow"]["p"].config(text=str(self.telemetry_data.get("elbow_pwm", "--")) + "%")
        
        # Wrist (DC Motor)
        self.tel_labels["Wrist"]["a"].config(text=str(self.telemetry_data.get("wrist", "--")))
        self.tel_labels["Wrist"]["r"].config(text=str(self.telemetry_data.get("wrist_raw", "--")))
        self.tel_labels["Wrist"]["p"].config(text=str(self.telemetry_data.get("wrist_pwm", "--")) + "%")

        # Gripper (Servo)
        self.tel_labels["Gripper"]["a"].config(text=str(self.telemetry_data.get("gripper", "--")))
        self.tel_labels["Gripper"]["r"].config(text="N/A")
        self.tel_labels["Gripper"]["p"].config(text="N/A")

    def add_slider(self, label_text, motor_id, min_val, max_val):
        frame = ttk.Frame(self.root)
        frame.pack(pady=5, fill='x', padx=20)
        
        ttk.Label(frame, text=label_text).pack(anchor='w')
        
        # Create scale
        scale = ttk.Scale(frame, from_=min_val, to=max_val, orient='horizontal')
        scale.pack(fill='x')
        
        # [NEW] Save reference so the remote can update this slider
        self.sliders[motor_id] = scale
        
        # Event binding: Only send when user releases mouse
        scale.bind("<ButtonRelease-1>", lambda event: self.send_command(motor_id, int(scale.get()))) 
    
    def add_limit_row(self, label, config_id):
        row = ttk.Frame(self.limits_frame)
        row.pack(fill='x', padx=5, pady=2)
        
        ttk.Label(row, text=label, width=15).pack(side=tk.LEFT)
        
        # Min Entry
        ttk.Label(row, text="Min:").pack(side=tk.LEFT)
        min_ent = ttk.Entry(row, width=5)
        min_ent.insert(0, "0")
        min_ent.pack(side=tk.LEFT, padx=5)
        
        # Max Entry
        ttk.Label(row, text="Max:").pack(side=tk.LEFT)
        max_ent = ttk.Entry(row, width=5)
        max_ent.insert(0, "180")
        max_ent.pack(side=tk.LEFT, padx=5)

        self.limit_entries[config_id] = {"min": min_ent, "max": max_ent}
        
        # Apply Button
        btn = ttk.Button(row, text="Apply", 
                   command=lambda: self.apply_limit(config_id, min_ent.get(), max_ent.get()))
        btn.pack(side=tk.LEFT, padx=10)

    def toggle_limits_menu(self):
        if self.show_limits.get():
            self.limits_frame.pack(pady=5, padx=20, fill='x')
        else:
            self.limits_frame.pack_forget()

    def apply_limit(self, config_id, min_str, max_str):
        try:
            min_val = int(min_str)
            max_val = int(max_str)
            
            if min_val < 0 or max_val > 180 or min_val >= max_val:
                print("Error", "Invalid values! ensure 0 <= Min < Max <= 180")
                return
            
            self.send_limit_config(config_id, min_val, max_val)
            print("Success", f"Limits updated for ID {config_id-10}")
            
        except ValueError:
            print("Error", "Please enter valid integers.")

    def on_close(self):
        self.running = False
        if self.ser: self.ser.close()
        self.root.destroy()

# --- MAIN ---
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControllerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()