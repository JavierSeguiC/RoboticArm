import tkinter as tk
from tkinter import ttk
import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM8'
BAUD_RATE = 115200

class RobotControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Manipulator Control")
        self.root.geometry("400x550")
        
        self.speed_val = 50 #default speed

        # Serial Connection
        self.ser = None
        self.connect_serial()

        # UI Layout
        self.create_widgets()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Wait for Arduino to reset
            print(f"Connected to {SERIAL_PORT}")
        except Exception as e:
            print(f"Error connecting: {e}")

    """ def send_command(self, motor_id, value):
        command = f"<{motor_id},{value}>"
        print(f"Sent: {command}") """

    def send_command(self, motor_id, angle):
        """
        Sends data in the format <ID, ANGLE, SPEED>
        """
        if self.ser and self.ser.is_open:
            # Construct command with the current global speed value
            command = f"<{motor_id},{angle},{self.speed_val}>"
            self.ser.write(command.encode())
            print(f"Sent: {command}")
        else:
            print(f"Serial not connected. Command would be: <{motor_id},{angle},{self.speed_val}>")

    def update_speed(self, val):
        self.speed_val = int(float(val))
        print(f"Speed set to: {self.speed_val}%")

    def create_widgets(self):
        # Title
        ttk.Label(self.root, text="Manual Control", font=("Arial", 16)).pack(pady=10)

        # Speed Control
        speed_frame = ttk.LabelFrame(self.root, text="Global Velocity Limit (%)")
        speed_frame.pack(pady=10, padx=20, fill='x')
        
        # Range 1-100. Default is set to 50 via variable
        speed_scale = ttk.Scale(speed_frame, from_=1, to=100, orient='horizontal', command=self.update_speed)
        speed_scale.set(50) # Set default to 50%
        speed_scale.pack(fill='x', padx=10, pady=5)

        # Base Control
        self.add_slider("Base Rotation", 1, 0, 180)
        
        # Elbow Control
        self.add_slider("Elbow Joint", 2, 0, 180)
        
        # Wrist Control
        self.add_slider("Wrist Joint", 3, 0, 180)

        # Gripper Control (Buttons instead of slider)
        frame = ttk.Frame(self.root)
        frame.pack(pady=10)
        ttk.Button(frame, text="Open Gripper", command=lambda: self.send_command(4, 90)).pack(side=tk.LEFT, padx=5)
        ttk.Button(frame, text="Close Gripper", command=lambda: self.send_command(4, 180)).pack(side=tk.LEFT, padx=5)

    def add_slider(self, label_text, motor_id, min_val, max_val):
        frame = ttk.Frame(self.root)
        frame.pack(pady=5, fill='x', padx=20)
        
        ttk.Label(frame, text=label_text).pack(anchor='w')
        
        # Slider calls send_command on change
        scale = ttk.Scale(frame, from_=min_val, to=max_val, orient='horizontal')
        scale.pack(fill='x')
        
        # Event binding: send command only when slider is released (to avoid flooding Arduino)
        scale.bind("<ButtonRelease-1>", lambda event: self.send_command(motor_id, int(scale.get())))
       


# --- MAIN ---
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControllerApp(root)
    root.mainloop()