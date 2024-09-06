#!/usr/bin/env python3
from pymavlink import mavutil
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import time

# Global değişkenler
lin_vel = 1500  # Başlangıç PWM değeri
steer_angle = 90  # Başlangıç direksiyon açısı değeri
fon = 1250
foff = 1800
bon = 1800
boff = 1000
ton = 1000
toff = 2000  # Takviye ve diferansiyel servo değerleri

def arm_drone():
    """Drone'u arm et."""
    print("Arming drone...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone Armed")

def disarm_drone():
    """Drone'u disarm et."""
    print("Disarming drone...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Drone Disarmed")

def angle_to_pwm(angle):
    """Açıyı PWM değerine dönüştür."""
    min_pwm = 1000  # Minimum PWM değeri
    max_pwm = 2000  # Maksimum PWM değeri
    min_angle = -30  # Minimum açı (-30 derece)
    max_angle = 30   # Maksimum açı (30 derece)

    # PWM aralığını ve açı aralığını hesapla
    pwm_range = max_pwm - min_pwm
    angle_range = max_angle - min_angle

    # Açıya bağlı PWM değerini hesapla
    pwm_value = min_pwm + (angle - min_angle) * (pwm_range / angle_range)
    pwm_value = int(pwm_value)
    
    # PWM sınırlarını kontrol et
    if pwm_value > max_pwm:
        pwm_value = max_pwm
    elif pwm_value < min_pwm:
        pwm_value = min_pwm
    
    return pwm_value

def send_pwm_to_esc(lin_pwm_value, ang_pwm_value, front_diff_pwm, back_diff_pwm, buffer_pwm):
    """PWM değerlerini Pixhawk'a gönder."""
    rc_channel_values = [ang_pwm_value, 0, lin_pwm_value, front_diff_pwm, back_diff_pwm, buffer_pwm, 0, 0]
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *[int(value) for value in rc_channel_values]
    )
    print(f"Sent PWM: lin_vel={lin_pwm_value}, ang_vel={ang_pwm_value}")

def callback_acker(msg):
    """AckermannDriveStamped mesajını işle ve Pixhawk'a gönder."""
    global lin_vel, steer_angle, fon, foff, bon, boff, ton, toff
    
    # Gelen hız ve direksiyon açılarını al
    speed = msg.drive.speed
    steering_angle = msg.drive.steering_angle

    print(f"Received speed: {speed}")
    print(f"Received steering_angle: {steering_angle}")

    # Direksiyon açısını PWM değerine dönüştür
    ang_vel = angle_to_pwm(steering_angle)
    
    # Hızı güncelle
    if speed != 0:
        lin_vel = 1500 + speed * 20  # Örnek hesaplama, hızın PWM'e dönüşümü
        lin_vel = max(min(lin_vel, 1560), 1440)  # Hızı sınırla
        print(f"New lin_vel: {lin_vel}")
    else:
        lin_vel = 1500  # Hız nötr konumda

    
    send_pwm_to_esc(lin_vel, ang_vel, boff, foff, ton)
    lin_vel = 1500

def ackermann_listener():
    """ROS node'unu başlat ve `ackermann_cmd` konusuna abone ol."""
    print("ackermann_listener node started")
    rospy.init_node('ackermann_listener', anonymous=True)
    rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, callback_acker)
    rospy.spin()

if __name__ == '__main__':
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
    master.wait_heartbeat()
    print("Heartbeat received, connected to Pixhawk")

    try:
        arm_drone()  # Drone'u arm et
        send_pwm_to_esc(1500, 1500, boff, foff, toff)
        ackermann_listener()  # ROS node'unu başlat
    except KeyboardInterrupt:
        disarm_drone()  # Program durdurulduğunda drone'u disarm et
    except Exception as e:
        print(f"An error occurred: {e}")
        disarm_drone()
    finally:
        disarm_drone()
