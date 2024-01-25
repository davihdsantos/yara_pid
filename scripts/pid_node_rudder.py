#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates, LinkStates
from simple_pid import PID
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import rad2deg

pub = rospy.Publisher('eboat/control_interface/rudder', Float32, queue_size=10)
pid_rudder = PID(-1, 0.0, 0.0)
pid_rudder.output_limits = (-90, 90)
heading_sp = Float32()
rudder = Float32()
eboat_heading_current = 0

def callback(data):
    eboat_name = 'eboat'
    eboat_link_index = data.name.index(eboat_name)
    eboat_heading_current = get_link_angle_deg(data.pose[eboat_link_index].orientation)
    pid_rudder.setpoint = heading_sp.data
    rudder.data = pid_rudder(eboat_heading_current)
    rudder.data = rudder.data
    pub.publish(rudder)
    # print(eboat_heading_current)

def callback_heading_sp(data):
    heading_sp.data = data.data
    # print(heading_sp)

def get_link_angle_deg (orientation):
    orientation_q = orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    # print(rudder.data)

    return rad2deg(yaw)
    
def listener():
    rospy.init_node('rudder_control', anonymous=True)
    rospy.Subscriber("/eboat/control_interface/heading_sp", Float32, callback_heading_sp)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()