#!/usr/bin/env python3
import time
from pymavlink import mavutil

def connect_pixhawk():
    connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    connection.wait_heartbeat()
    print("Bağlantı başarılı!")
    return connection

def get_scaled_imu_data(connection):
    while True:
        message = connection.recv_match(type='SCALED_IMU2', blocking=True)
        if not message:
            continue

        print(f"Scaled IMU Data: xacc: {message.xacc}, yacc: {message.yacc}, zacc: {message.zacc}, "
              f"xgyro: {message.xgyro}, ygyro: {message.ygyro}, zgyro: {message.zgyro}, "
              f"xmag: {message.xmag}, ymag: {message.ymag}, zmag: {message.zmag}")
        time.sleep(1)

if __name__ == '__main__':
    pixhawk_connection = connect_pixhawk()
    try:
        get_scaled_imu_data(pixhawk_connection)
    except KeyboardInterrupt:
        print("Kapatılıyor...")
