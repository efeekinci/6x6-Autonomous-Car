#!/usr/bin/env python3
from pymavlink import mavutil
import time

# MAVLink bağlantısını kurun (Pixhawk'un bağlı olduğu doğru portu belirtin)
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# MAVLink mesajlarını beklemek için sisteme komut gönderin
connection.wait_heartbeat()

while True:
    # GPS2_RAW veya GPS_RTK mesajını bekle
    msg = connection.recv_match(type=['GPS2_RAW', 'GPS_RTK'], blocking=True)
    if msg:
        if msg.get_type() == 'GPS2_RAW':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            print(f"RTK GPS Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
            time.sleep(1)
        elif msg.get_type() == 'GPS_RTK':
            baseline_heading = msg.baseline_heading / 100000  # 1e5 precision
            accuracy = msg.accuracy
            nsats = msg.nsats
            print(f"RTK Baseline Heading: {baseline_heading}, Accuracy: {accuracy}, Number of Satellites: {nsats}")
            time.sleep(1)
