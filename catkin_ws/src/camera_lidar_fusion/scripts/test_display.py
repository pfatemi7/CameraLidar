#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tkinter as tk
from tkinter import messagebox
import threading
import time

class DisplayTest:
    def __init__(self):
        rospy.init_node('display_test', anonymous=True)
        
        # Create a simple GUI window
        self.root = tk.Tk()
        self.root.title("Camera-LiDAR Fusion Test")
        self.root.geometry("400x300")
        self.root.configure(bg='lightblue')
        
        # Add some widgets
        label = tk.Label(self.root, text="🎯 Camera-LiDAR Fusion System", 
                        font=("Arial", 16), bg='lightblue')
        label.pack(pady=20)
        
        status_label = tk.Label(self.root, text="System Status: Running", 
                               font=("Arial", 12), bg='lightgreen')
        status_label.pack(pady=10)
        
        # Add a button
        button = tk.Button(self.root, text="Test Button", 
                          command=self.show_message, bg='orange')
        button.pack(pady=10)
        
        # Add ROS status
        self.ros_label = tk.Label(self.root, text="ROS: Initializing...", 
                                 font=("Arial", 10), bg='lightblue')
        self.ros_label.pack(pady=10)
        
        # Start ROS monitoring in a separate thread
        self.ros_thread = threading.Thread(target=self.monitor_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        rospy.loginfo("Display test window created")
    
    def show_message(self):
        messagebox.showinfo("Test", "Display is working! 🎉")
    
    def monitor_ros(self):
        while not rospy.is_shutdown():
            try:
                # Check if ROS topics are active
                topics = rospy.get_published_topics()
                if topics:
                    self.ros_label.config(text=f"ROS: {len(topics)} topics active")
                else:
                    self.ros_label.config(text="ROS: No topics found")
            except:
                self.ros_label.config(text="ROS: Error")
            
            time.sleep(2)
    
    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            rospy.loginfo("Display test interrupted")
        finally:
            self.root.destroy()

if __name__ == '__main__':
    try:
        test = DisplayTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
