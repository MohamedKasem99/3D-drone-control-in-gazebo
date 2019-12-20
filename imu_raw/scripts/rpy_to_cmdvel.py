#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist,Vector3Stamped, Vector3 
def callback(pub, gl, ga, data):
    x = data.vector.x
    y = data.vector.y
    pub.publish(Twist(Vector3(x*gl, 0, 0), Vector3(0, 0, y*ga)))
def main():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.init_node("rpy_to_cmdvel")
    gl = rospy.get_param('~g_linear', 1)
    ga = rospy.get_param('~g_angular', 1)
    rospy.loginfo(f"Linear gain is {gl}")
    rospy.loginfo(f"Angular gain is {ga}")
    rospy.Subscriber("/imu/rpy/filtered", Vector3Stamped, lambda data: callback(pub, gl, ga,data))
    if rospy.has_param("~g_linear"):
        rospy.delete_param("~g_linear")
    if rospy.has_param("~g_angular"):
        rospy.delete_param("~g_angular")
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

