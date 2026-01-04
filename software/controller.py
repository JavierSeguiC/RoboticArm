import tkinter as tk
from tkinter import ttk
import serial
import time
import threading

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
            "elbow_ang": 0, "elbow_raw": 0,
            "wrist_ang": 0, "wrist_raw": 0
        }

        self.create_widgets()
        self.connect_serial()
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Wait for Arduino to reset
            print(f"Connected to {SERIAL_PORT}")

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
                        
                        # --- PARSE TELEMETRY ---
                        if line.startswith("STATUS:"):
                            # Format: STATUS:ElbowAngle,ElbowRaw,WristAngle,WristRaw
                            parts = line.split(":")[1].split(",")
                            if len(parts) == 4:
                                self.telemetry_data["elbow_ang"] = parts[0]
                                self.telemetry_data["elbow_raw"] = parts[1]
                                self.telemetry_data["wrist_ang"] = parts[2]
                                self.telemetry_data["wrist_raw"] = parts[3]
                                self.update_telemetry_ui()
                        elif line:
                            print(f"[ARDUINO]: {line}") 
                except Exception as e:
                    print(f"Serial Read Error: {e}")
            time.sleep(0.01)

    def send_command(self, motor_id, angle):
        # Update local target memory
        if motor_id == 1: self.target_base = angle
        elif motor_id == 2: self.target_elbow = angle
        elif motor_id == 3: self.target_wrist = angle
        elif motor_id == 4: self.target_gripper = angle
        
        # Send to Arduino
        if self.ser and self.ser.is_open:
            command = f"<{motor_id},{angle},{self.speed_val}>"
            self.ser.write(command.encode())
            print(f"Command: {command}")

            self.update_telemetry_ui()

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
        ttk.Label(self.telemetry_frame, text="Raw (0-1023)", font=("Arial", 9, "bold")).grid(row=0, column=3, padx=5)

        # Grid Data Labels (We keep references to update them later)
        self.tel_labels = {}
        joints = ["Base", "Elbow", "Wrist", "Gripper"]
        for i, joint in enumerate(joints):
            row = i + 1
            ttk.Label(self.telemetry_frame, text=joint).grid(row=row, column=0, pady=2)
            
            # Create labels for Target, Actual, Raw
            l_target = ttk.Label(self.telemetry_frame, text="--")
            l_target.grid(row=row, column=1)
            
            l_actual = ttk.Label(self.telemetry_frame, text="--")
            l_actual.grid(row=row, column=2)
            
            l_raw = ttk.Label(self.telemetry_frame, text="--")
            l_raw.grid(row=row, column=3)
            
            self.tel_labels[joint] = {"t": l_target, "a": l_actual, "r": l_raw}

        # --- 2. CONTROLS SECTION ---
        ttk.Label(self.root, text="Manual Control", font=("Arial", 16)).pack(pady=10)

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
        ttk.Button(frame, text="Open Gripper", command=lambda: self.send_command(4, 90)).pack(side=tk.LEFT, padx=5)
        ttk.Button(frame, text="Close Gripper", command=lambda: self.send_command(4, 180)).pack(side=tk.LEFT, padx=5)

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
        # Update Targets (From our local commands)
        self.tel_labels["Base"]["t"].config(text=str(self.target_base))
        self.tel_labels["Elbow"]["t"].config(text=str(self.target_elbow))
        self.tel_labels["Wrist"]["t"].config(text=str(self.target_wrist))
        self.tel_labels["Gripper"]["t"].config(text=str(self.target_gripper))

        # Update Actuals (From Arduino Telemetry)
        # Note: Base and Gripper are servos (Open Loop), so Actual = Target, Raw = N/A
        self.tel_labels["Base"]["a"].config(text=str(self.target_base))
        self.tel_labels["Base"]["r"].config(text="N/A")
        
        self.tel_labels["Gripper"]["a"].config(text=str(self.target_gripper))
        self.tel_labels["Gripper"]["r"].config(text="N/A")

        # DC Motors (Closed Loop)
        self.tel_labels["Elbow"]["a"].config(text=self.telemetry_data["elbow_ang"])
        self.tel_labels["Elbow"]["r"].config(text=self.telemetry_data["elbow_raw"])
        
        self.tel_labels["Wrist"]["a"].config(text=self.telemetry_data["wrist_ang"])
        self.tel_labels["Wrist"]["r"].config(text=self.telemetry_data["wrist_raw"])

    def add_slider(self, label_text, motor_id, min_val, max_val):
        frame = ttk.Frame(self.root)
        frame.pack(pady=5, fill='x', padx=20)
        
        ttk.Label(frame, text=label_text).pack(anchor='w')
        
        # Slider calls send_command on change
        scale = ttk.Scale(frame, from_=min_val, to=max_val, orient='horizontal')
        scale.pack(fill='x')
        
        # Event binding: send command only when slider is released (to avoid flooding Arduino)
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
                retur
            
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