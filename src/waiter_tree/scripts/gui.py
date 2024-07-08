#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ROS2 Node
class ControlPublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.publisher_ = self.create_publisher(String, 'control_topic', 10)
    
    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

# Tkinter GUI
class ControlGUI:
    def __init__(self, root, publisher):
        self.publisher = publisher

        root.title("Control GUI")
        root.geometry("400x300")
        root.configure(bg='#0e1f56')  # Dark blue background

        style = ttk.Style()
        style.configure("TButton", font=('Helvetica', 14), padding=15)
        style.map("TButton",
                foreground=[('pressed', 'black'), ('active', 'white')],
                background=[('pressed', '!disabled', 'white'), ('active', 'blue')])

        # Add a title label
        title_label = tk.Label(root, text="Control Panel", font=('Helvetica', 18, 'bold'), bg='#0e1f56', fg='white')
        title_label.pack(pady=20)

        # Buttons
        button1 = ttk.Button(root, text="Control 1", command=lambda: self.button_action("Control 1"))
        button1.pack(pady=10)

        button2 = ttk.Button(root, text="Control 2", command=lambda: self.button_action("Control 2"))
        button2.pack(pady=10)

        button3 = ttk.Button(root, text="Control 3", command=lambda: self.button_action("Control 3"))
        button3.pack(pady=10)

        button4 = ttk.Button(root, text="Control 4", command=lambda: self.button_action("Control 4"))
        button4.pack(pady=10)

    def button_action(self, control):
        print(f"{control} pressed")
        self.publisher.publish_message(control)

def main(args=None):
    rclpy.init(args=args)

    # Create the ROS 2 publisher node
    control_publisher = ControlPublisher()

    # Start the Tkinter GUI
    root = tk.Tk()
    gui = ControlGUI(root, control_publisher)
    root.mainloop()

    # Shutdown ROS 2
    control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""        
root = tk.Tk()
root.title("control GUI")
root.geometry("400x300")
root.configure(bg='#0e1f56')  # Dark blue background

style = ttk.Style()
style.configure("TButton", font=('Helvetica', 14), padding=15)
style.map("TButton",
          foreground=[('pressed', 'black'), ('active', 'white')],
          background=[('pressed', '!disabled', 'white'), ('active', 'blue')])

# Add a title label
title_label = tk.Label(root, text="Control Panel", font=('Helvetica', 18, 'bold'), bg='#0e1f56', fg='white')
title_label.pack(pady=20)

# DEF functionss
def button1_action():
    print("Button 1 pressed")

def button2_action():
    print("Button 2 pressed")

def button3_action():
    print("Button 3 pressed")

def button4_action():
    print("Button 4 pressed")

# buttons
button1 = ttk.Button(root, text="Control 1", command=button1_action)
button1.pack(pady=10)

button2 = ttk.Button(root, text="Control 2", command=button2_action)
button2.pack(pady=10)

button3 = ttk.Button(root, text="Control 3", command=button3_action)
button3.pack(pady=10)

button4 = ttk.Button(root, text="Control 4", command=button4_action)
button4.pack(pady=10)

root.mainloop()
"""

