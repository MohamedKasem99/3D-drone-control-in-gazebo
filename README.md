# Wireless glove controller for parrot drones 
In this project, we implement a 3d interactive glove controller that is used to control a parrot drone in Gazebo virtual environment. Using an esp8622 module (NodeMCU) and an IMU sensor (MPU6050),we obtain the orientation data then send it over UDP for semi-realtime communication. The data is then parsed, filtered and published to a ROS network where Gazebo and Rviz software can access and visualize it in 3d Space. The project is fully implemented in ROS.

## Design choices

**Embedded system**: We chose esp8622 (Node MCU Breakout) instead of Arduino because of the added wifi functionality and ease of programming. This allowed us to communicate the data over wifi with no need for serial wired communication which means better user experience and more reliable data transmission. 

**Communication protocol**: UDP was clearly more suitable than TCP since it offers lower communication latency and therefore could provide a semi-real time interaction. Despite the fact that UDP is less reliable than TCP since it might drop packets of data every now and then, it was still a very viable option for our application due to the high sampling rate that we operate on. That means we can tolerate occasional drops in control messages with little to no effect on the user experience. 

**ROS**: We chose ROS because of its wide support, efficiency and consistency. ROS offers a fairly simple and straight forward platform for data communication and module integration. The publisher-subscriber system has eliminated the overhead of integrating all the modules and communicating the data between them. Also, due to its consistency and wide support, we could use ready-made packages from github with very minor modifications. 

## Schematic and Hardware

![](/media/kasem/Happy_place/University stuff/Year #4/Fall 2019/Embedded Systems/Projects/Final project/Finalized codes/3D-drone-control-in-gazebo/images/Schematic.png)

<img src="/media/kasem/Happy_place/University stuff/Year #4/Fall 2019/Embedded Systems/Projects/Final project/Finalized codes/3D-drone-control-in-gazebo/images/Hardware.jpg" alt="Hardware" style="zoom:50%;" />

#### Note

In case of failure to reproduce the project, try using the same versions for the Arduino IDE and its ESP board manager. Our versions were: 
Arduino IDE version == 1.8.9
ESP board manger == 2.6.0

For more details follow this [tutorial](https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/) on how to upload Arduino C code to NodeMCU with Arduino IDE. However, when it comes to choosing the board manager version, refer to the version mentioned above. 

## Steps to run the whole project

1- create and setup the catkin workspace (skip if you already have your own or create another with a different name).

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

2- clone this repository into your src folder

```bash
$ git clone 
```



