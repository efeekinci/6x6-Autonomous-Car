# 6x6 Autonomous Car
This is a repository of ROMER Summer Internship 2024. The project is reconstructing and restarting the 6x6 radio controlled car into ROS compatible reseach platform made by Özgür Gülsuna, Emre Dağ, Mustafa Akbaba for graduation project in 2021-2022. For original repository [click here](https://github.com/ozgurgulsuna/ME462-MechatronicDesign.git)


### Group Members
- [Göktuğhan Efe Ekinci](https://github.com/efeekinci)
- [Eren Kazaz](https://github.com/ernkazaz)
- Yiğit Adaş
## Contents
- [About Project](#about-project)
- [Vehicle Hardware](#vehicle-hardware)
- [Vehicle Software](#vehicle-software)
- [Arduino Control Software](#arduino-control-software)
- [Car Control](#car-control)
- [Setup and Usage](#setup-and-usage)
- [Results](#results)
- [Future Works](#future-works)

## About Project
This project was conducted as part of an internship at ROMER. The goal is to enable a 6x6 vehicle to move autonomously. The vehicle uses various sensors and control mechanisms to detect obstacles and determine its path.

## Vehicle Hardware
The main components used in this project are:
- **6x6 Vehicle Chassis**: Mechanical structure and wheel system.
- **Nvidia Jetson Nano**: Main control unit and image processing.
- **Raspberry Pi Pico**: Hall Effect sensor encoder.
- **Arduino Nano**: Motor control.
- **Sensors**: LiDAR, camera module, hall effect, ultrasonic.
- **Traxxas XL-5 ESC**: Electronic speed control unit.
- **Others**: Battery, cables, connectors, DC-DC converter.

## Vehicle Software
This section explains the software and algorithms that enable the vehicle to operate autonomously.
(Buraya da ne yazacağımızdan emin değilim silinebilir belki.)
### Main Control Software
(Bunlardan emin değilim.)
The main control software running on Nvidia Jetson manages the vehicle's movement and sensor data. Key components of the software are:
- **Sensor Data Collection**: Processing data from sensors.
- **Image Processing**: Processing camera data using the YOLO model.
- **Motion Planning**: Determining the vehicle's path through algorithms.
- **Sensor Reading**: Reading data from ultrasonic sensors and LiDAR.

### Arduino Control Software
The Arduino controls the motors and sensors. Its main functions are:
- **Motor Control**: Controlling the speed and direction of the motors.

## Car Control
Methods used to control the vehicle:
- **Manual Control**: Manual control via keyboard over SSH.
- **Autonomous Control**: Autonomous movement through software algorithms. (belki repoyu inceleyip kullanılan yapay zeka metodunu elaborate ederiz proje üzerinde)

## Setup and Usage
**Manual Control**

Follow these steps to run the project in your environment:
- From the keyboardControl folder upload the main_code.ino file to your Arduino Nano
- Connect the Jetson Nano from your computer with SSH
   ```sh
   ssh -X username@jetson-ip-address
   ```
- Run the arduino_talker.py and ros_keyboard.py files from terminal.

## Results
As a result of this project, the 6x6 vehicle was successfully made autonomous. The vehicle was able to detect various obstacles and navigate around them to follow the desired path. (gibisinden şeyler, incele)

For further information: [https://github.com/ozgurgulsuna/ME462-MechatronicDesign/tree/main](https://github.com/ozgurgulsuna/ME462-MechatronicDesign.git)
### Requirements
- Nvidia Jetson
- Arduino
- Sensors and motors
### Future Works

  
