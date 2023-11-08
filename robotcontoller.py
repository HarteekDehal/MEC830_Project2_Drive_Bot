import time
import serial
import tkinter as tk
import threading

def main():
    serial_port = SerialPort(port='COM5', baud_rate=9600)
    input_handler = InputHandler()
    robot = RobotController(serial_port)
    
    # Robot subscribes to input_handler updates
    input_handler.register_observer(robot)
    root = tk.Tk()
    app = RobotGUI(root, robot, input_handler)
    root.mainloop()
    
class SerialPort:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud = baud_rate
        self.serial_connection = None

    def connect(self):
        self.serial_connection = serial.Serial(port=self.port, baudrate=self.baud)
        time.sleep(2)
        if self.serial_connection.is_open:
            return True
        else:
            return False

    def send_data(self, data):
        self.serial_connection.write(data.encode())

    def read_data(self):
        return self.serial_connection.readline().decode('utf-8').strip()

    def disconnect(self):
        self.serial_connection.close()

class RobotController:
    def __init__(self, serial_port):
            self.serial_port = serial_port
            self.is_connected = self.serial_port.connect()
            self.stop_condition = False

    def _send_command(self, command):
        # Send the command to the robot via the SerialPort instance
        if self.is_connected:
            self.serial_port.send_data(command)
        else:
            print("Connection not established.")
    
    def update(self, command=None, speed=None):
        if command:
            self.command = command
        if speed:
            self.speed = speed
            
    def read_feedback(self):
        if self.serial_port.serial_connection.inWaiting() > 0:
            return self.serial_port.read_data()
        return None
            
    def manual_control(self):
        self.command = "S"
        self.speed = None
        def control_loop():
            while not self.stop_condition:
                try:
                    self._send_command(self.command)
                    time.sleep(0.1)  # Adjust as needed         
                except KeyboardInterrupt:
                    print("Manual control has been terminated.")
                    break
                  
            self.stop_manual_control()
                    
        self.control_thread = threading.Thread(target=control_loop)
        self.control_thread.start()
        
    def stop_manual_control(self):
        self._send_command("S")
        self.stop_condition = True
        if self.control_thread.is_alive():
            self.control_thread.join()
          
class Observable:
    def __init__(self):
        self.observers = []

    def register_observer(self, observer):
        if observer not in self.observers:
            self.observers.append(observer)

    def unregister_observer(self, observer):
        if observer in self.observers:
            self.observers.remove(observer)

    def notify_observers(self, *args, **kwargs):
        for observer in self.observers:
            observer.update(*args, **kwargs)
    
class InputHandler(Observable):
    def __init__(self):
        super().__init__()
        self.command = None
        self.speed = None
        
        # WASD + Spacebar
        self.command_dict = {
            "w": "F",
            "s": "B",
            "a": "L",
            "d": "R",
            "space": "S"
        }
        
    def handle_keypress(self, event):
        key = event.keysym.lower()
        if key in self.command_dict:
            self.command = self.command_dict[key]
            self.notify_observers(command=self.command, speed=self.speed)
        else:
            print("Invalid command")
        
class RobotGUI:
    def __init__(self, master, robot_controller, input_handler =None, widget_manager=None):
        self.master = master
        self.robot_controller = robot_controller
        self.input_handler = input_handler or InputHandler()
        self.widget_manager = widget_manager or WidgetManager()
        self.master.title("Robot Controller")
        self.setup_gui()
        self.master.geometry("300x300")
        
        
    def setup_gui(self):
        self.manual_control_button = tk.Button(self.master, text="Manual Control", command=self.enter_manual_control_mode)
        self.manual_control_button.pack()
            
    def enter_manual_control_mode(self):
        self.master.unbind('<KeyPress>')
        self.widget_manager.configure_widgets_for_manual_control(self.master, self.robot_controller.stop_manual_control)
        self.master.bind('<KeyPress>', self.input_handler.handle_keypress)
        self.master.bind('<Escape>', lambda event: self.robot_controller.stop_manual_control())

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.robot_controller.manual_control()

    def update_visual_feedback(self):
        feedback = self.robot_controller.read_feedback()
        if feedback:
            feedback_label = self.widget_manager.get_widget("feedback_label")
            if feedback_label:
                feedback_label.config(text=f"Robot says: {feedback}")
                    

    def on_closing(self):
        self.input_handler.unregister_observer(self.robot_controller)
        self.robot_controller.disconnect()
        self.master.destroy()        

class WidgetManager:
    def __init__(self):
        self.widgets = {}

    def add_widget(self, name, widget):
        self.widgets[name] = widget

    def configure_widgets_for_manual_control(self, master, stop_callback):
        # Clear all current widgets first
        for widget in self.widgets.values():
            widget.pack_forget()

        # Create specific widgets for manual control
        feedback_label = tk.Label(master, text="Waiting for feedback...")
        feedback_label.pack()
        speed_entry = tk.Entry(master) # make this a spinbox
        speed_entry.pack()
        stop_button = tk.Button(master, text="Stop", command=stop_callback)
        stop_button.pack()

        self.widgets = {
            "feedback_label": feedback_label,
            "speed_entry": speed_entry,
            "stop_button": stop_button,
        }

    def get_widget(self, name):
        return self.widgets.get(name)    

if __name__ == "__main__":
    main()
