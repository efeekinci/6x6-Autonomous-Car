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
This project was conducted as part of an internship at ROMER. The goal is to enable a 6x6 vehicle to move autonomously. The purpose of the vehicle is to use various sensors to create a map using SLAM algorithm and be able to autonomously drive in the map that is already created.

## Vehicle Hardware
The main components used in this project are:
- **6x6 Vehicle Chassis**: Mechanical structure and wheel system.
- **Nvidia Jetson Nano**: Main control unit and image processing.
- **Raspberry Pi Pico**: Hall Effect sensor encoder.
- **Pixhawk 2.1 Cube Black**: Autopliot and motor control.
- **Emlid Reach M+**: RTK GNSS Module. (Currently not used)
- **Sensors**: LiDAR, camera module, hall effect, IMU(Pixhawk), RTK(Emlid Reach M+).
- **Traxxas XL-5 ESC**: Electronic speed control unit.
- **Others**: Battery, cables, connectors, DC-DC converter.

## Vehicle Software
This section explains the software and algorithms that enable the vehicle to operate autonomously.
### Main Control Software
The main control software running on Nvidia Jetson manages the vehicle's movement and sensor data. Key components of the software are:
- **Sensor Data Collection**: Processing data from sensors.
- **Motor Control**: Controlling the speed and direction of the motors through Pixhawk.
- **Mapping**: Creating a map using SLAM algorithm.
- **Motion Planning**: Determining the vehicle's path through algorithms.

## Car Control
Methods used to control the vehicle:
- **Manual Control**: Manual control. This type of control is used for creating a map with gmapping (SLAM) algorithm.
- **Autonomous Control**: Autonomous movement through software algorithms.

## Setup and Usage
**Manual Control**

Follow these steps to run the project in your environment:
- Connect the Jetson Nano from your computer with SSH
   ```sh
   ssh -X username@jetson-ip-address
   ```
- Run the motorControl.py and ros_keyboard.py files from terminal.
- Run the g_slam node from terminal.
  ```sh
   roslaunch g_slam gmapping.launch
   ```
- After the mapping is done save the map
  ```sh
   rosrun map_server map_saver -f map

For autonomous driving the user should change the map file to the desired map file via entering the name in .yaml file

**Autonomous Control**

Also follow these steps to run the project in your environment:
- Connect the Jetson Nano from your computer with SSH
   ```sh
   ssh -X username@jetson-ip-address
   ```
- Run Alize to initialize the robot.
   ```sh
   roslaunch Alize start.launch
   ```
- Run alize_2dnav to open rviz and start path planning.
   ```sh
   roslaunch alize_2dnav move_base.launch
   ```
- Initialize motorAutonom.py ros node to start autonomous drive. (This code may need slight adjustments to motor pwm values depending on the calibration of esc)

## Results
As a result of this project, the 6x6 vehicle was successfully made autonomous. The vehicle was able to create a map, detect various obstacles and navigate around them to follow the desired path. You can see the internship video [here](https://youtu.be/J0ETjmS26sI).

For further information: [https://github.com/ozgurgulsuna/ME462-MechatronicDesign/tree/main](https://github.com/ozgurgulsuna/ME462-MechatronicDesign.git)
### Requirements
- Nvidia Jetson
- Pixhawk 2.1 Cube Black
- Sensors and motors
### Future Works
   The car has suitable parts for outdoor drive such as RTK and IMU sensors. Moreover, there is also a slot for the camera so, the car is suitable for outdoor drive and image processing.
  
