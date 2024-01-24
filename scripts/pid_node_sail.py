#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32, Float32MultiArray

pub = rospy.Publisher('eboat/control_interface/sail', Float32, queue_size=10)

def callback(data):
    abs_wind = abs(data.data[4])
    tmp = Float32()
    tmp.data = abs((abs_wind / 2) - 90)
    pub.publish(tmp)
    
def listener():
    rospy.init_node('pid_control', anonymous=True)
    rospy.Subscriber("/eboat/mission_control/observations", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()