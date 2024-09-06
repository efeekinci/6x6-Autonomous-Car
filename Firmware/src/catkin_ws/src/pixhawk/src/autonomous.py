#!/usr/bin/env python3
from pymavlink import mavutil

# Pixhawk bağlantısını başlat
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()

# Tam gaz PWM değerini gönder (genellikle 2000)
high_pwm = 1700
rc_channel_values = [1500, 0, high_pwm, 0, 0, 0, 0, 0]
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_channel_values
)
print("High PWM (full throttle) sent")

low_pwm = 1300
rc_channel_values = [1500, 0, low_pwm, 0, 0, 0, 0, 0]
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    *rc_channel_values
)
print("Low PWM (throttle off) sent")