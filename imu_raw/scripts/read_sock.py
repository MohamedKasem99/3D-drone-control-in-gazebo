#!/usr/bin/env python3
import socket
import sys
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from time import sleep
seq = 0
def assign_data(imu_msg, data):
    global seq
    # Add the data to imu message and increment seq
    imu_msg.linear_acceleration.x = float(data[0])
    imu_msg.linear_acceleration.y = float(data[1])
    imu_msg.linear_acceleration.z = float(data[2])
    imu_msg.angular_velocity.x = float(data[3])
    imu_msg.angular_velocity.y = float(data[4])
    imu_msg.angular_velocity.z = float(data[5])
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu"
    imu_msg.header.seq = seq
    seq +=1
    return
def main():
    # 1. Init a Publisher object with name that the 
    #    complementary filter expects
    pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    # 2. Init a ROS Node
    rospy.init_node('imu_raw_publisher')
    rospy.loginfo("made imu_raw_publisher node")
    # 3. Specify rate at 1000 Hz
    rate = rospy.Rate(1000)
    # 4. Create a UDP Socket
    txSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    txSocket.setblocking(0)
    # 5. Init an Imu Message
    imu_msg = Imu()
    rospy.loginfo("Created Socket and IMU object")
    # 6. Try sending to NodeMCU and wait for ACKnowledgement
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Sending to UDP Node")
            # 6.1 Send a space to initiate conversation
            txSocket.sendto("  ".encode(),("192.168.1.90",8888))
            sleep(.2)
            # 6.2 Wait for acknowledgment signal ir try again
            data, addr = txSocket.recvfrom(1024)   
            data = data.decode().split(',')
            if data == ["acknowledged\r\n"]:
                rospy.loginfo("Received acknowledgement")
                break
        except socket.error as msg:
            pass
    rospy.loginfo("Success")
    # 7. Read Data and populate the Imu and publish it
    while not rospy.is_shutdown():
        try:
            # 7.1 recive and decode and split by comme ","
            data, addr = txSocket.recvfrom(1024)   
            data = data.decode().split(',')
            if data == ["acknowledged\r\n"]:
                rospy.loginfo("Received acknowledgement")
                continue
            # 7.2 Fill the imu message
            assign_data(imu_msg, data)
            # 7.3 Publish the message
            pub.publish(imu_msg)
            # 7.4 Sleep the repeat
            rate.sleep()
        except BlockingIOError:
            pass
        except:
            rospy.loginfo(f"some error happend : {sys.exc_info()[0].__name__}: {sys.exc_info()[1].args}")
            pass
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

