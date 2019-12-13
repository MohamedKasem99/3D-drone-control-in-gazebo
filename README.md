# 3D-drone-control-in-gazebo
Implementing a 3D glove controller. Using an esp8622 module (NodeMCU) and an IMU sensor (MPU6050), the orientation data is sent over UDP for semi-realtime communication. The data is then parsed, filtered and published to a ROS network where Gazebo and Rviz software can access and visualize it in 3d Space. The project is fully implemented in ROS.
