#!/usr/bin/env python3
from pymavlink import mavutil
import time
import rospy
from geometry_msgs.msg import Twist



# Global değişkenler
lin_vel = 1500  # Başlangıç PWM değeri
steer_angle = 90  # Başlangıç direksiyon açısı değeri
fon = 1250, foff = 1800, bon = 1800, boff = 1270, ton = 1220, toff = 1600  # Diferansiyel ve takviye servo değerleri

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
    min_angle = 60  # Minimum açı
    max_angle = 120  # Maksimum açı

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

def callback_car_vel(msg):
    """ROS üzerinden gelen hız ve direksiyon açısını işle."""
    global lin_vel, steer_angle, fon, foff, bon, boff, ton, toff
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    print(f"Received linear.x: {linear_x}")
    print(f"Received angular.z: {angular_z}")

    # Direksiyon açısını güncelle
    steer_angle += angular_z
    steer_angle = max(min(steer_angle, 120), 60)  # Açıyı sınırla
    ang_vel = angle_to_pwm(steer_angle)
    
    # Hızı güncelle
    if linear_x != 0:
        lin_vel += linear_x
        lin_vel = max(min(lin_vel, 1600), 1400)  # Hızı sınırla
        print(f"New lin_vel: {lin_vel}")
    else:
        lin_vel = 1500  # Hız nötr konumda

    # PWM değerlerini Pixhawk'a gönder
    send_pwm_to_esc(lin_vel, ang_vel, foff, boff, ton)

def car_vel():
    """ROS node'unu başlat ve `cmd_vel` konusuna abone ol."""
    print("car_vel node started")
    rospy.init_node('car_vel', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback_car_vel)
    rospy.spin()

if __name__ == '__main__':
    # Pixhawk ile bağlantıyı kur
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    print("Heartbeat received, connected to Pixhawk")

    try:
        arm_drone()  # Drone'u arm et
        send_pwm_to_esc(1500,1500,foff,boff,ton)
        car_vel()    # ROS node'unu başlat
    except KeyboardInterrupt:
        disarm_drone()  # Program durdurulduğunda drone'u disarm et
    except Exception as e:
        print(f"An error occurred: {e}")
        disarm_drone()
    finally:
        disarm_drone()
