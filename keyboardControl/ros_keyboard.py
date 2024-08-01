#!/usr/bin/env python
from pynput import keyboard
import threading
import serial
import time
import rospy
import os
from geometry_msgs.msg import Twist

# Global variables for tracking pressed keys and velocities
key_state = {'w': False, 'a': False, 's': False, 'd': False}
lin_vel, ang_vel = 0, 0

def pub_commands():
    print("inside pub")
    global lin_vel, ang_vel
    move_cmd = Twist()

    # Check combinations and set velocities accordingly
    if key_state['w'] and key_state['d']:
        move_cmd.linear.x = abs(lin_vel)
        move_cmd.angular.z = abs(ang_vel)
    elif key_state['w'] and key_state['a']:
        move_cmd.linear.x = abs(lin_vel)
        move_cmd.angular.z = -abs(ang_vel)
    elif key_state['s'] and key_state['a']:
        move_cmd.linear.x = -abs(lin_vel)
        move_cmd.angular.z = -abs(ang_vel)
    elif key_state['s'] and key_state['d']:
        move_cmd.linear.x = -abs(lin_vel)
        move_cmd.angular.z = abs(ang_vel)
    elif key_state['w']:
        move_cmd.linear.x = abs(lin_vel)
    elif key_state['s']:
        move_cmd.linear.x = -abs(lin_vel)
    elif key_state['a']:
        move_cmd.angular.z = -abs(ang_vel)
    elif key_state['d']:
        move_cmd.angular.z = abs(ang_vel)
    
    pub.publish(move_cmd)

def on_press(key):
    global lin_vel, ang_vel
    try:
        print(lin_vel, ang_vel)
        if key.char in key_state:
            key_state[key.char] = True
        
        if key.char == 'e':
            lin_vel += 10
        elif key.char == 'q':
            lin_vel -= 10
        elif key.char == 'z':
            ang_vel += 10
        elif key.char == 'c':
            ang_vel -= 10       
        pub_commands()

    except AttributeError:
        pass

def on_release(key):
    if key.char in key_state:
        key_state[key.char] = False
    pub_commands()

if __name__ == '__main__':
    print("Now in main 1")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('pub_commands', anonymous=True)
    
    # Start listening to keyboard events
    print("Now in main 2")
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Keep the script running
    rospy.spin()