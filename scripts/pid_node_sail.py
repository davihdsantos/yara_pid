#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32, Float32MultiArray, Int16

pub = rospy.Publisher('eboat/control_interface/sail', Float32, queue_size=10)
pub_motor = rospy.Publisher('eboat/control_interface/propulsion', Int16, queue_size=10)

def callback(data):
    abs_wind = abs(data.data[4])
    tmp = Float32()
    tmp.data = abs((abs_wind / 2) - 90)
    if(abs_wind > 150 and abs_wind < 180):
        pub_motor.publish(Int16(1))
    else:
        pub_motor.publish(Int16(0))    

    pub.publish(tmp)
    
def listener():
    rospy.init_node('sail_control', anonymous=True)
    rospy.Subscriber("/eboat/mission_control/observations", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()