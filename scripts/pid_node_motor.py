#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Int16
from gazebo_msgs.msg import ModelStates, LinkStates
from simple_pid import PID
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import rad2deg

pub = rospy.Publisher('eboat/control_interface/propulsion', Int16, queue_size=10)
pid_speed = PID(-1, -0.05, 0.0)
pid_speed.output_limits = (0, 3)
heading_sp = Float32()
eboat_heading_current = 0
speed_sp = Int16(3)

def callback(data):
    eboat_name = 'eboat'
    eboat_link_index = data.name.index(eboat_name)
    eboat_heading_current = get_link_angle_deg(data.pose[eboat_link_index].orientation)
    pid_rudder.setpoint = heading_sp.data
    rudder.data = pid_rudder(eboat_heading_current)
    rudder.data = rudder.data
    pub.publish(rudder)
    # print(eboat_heading_current)

def callback_speed_sp(data):
    speed_sp.data = data.data
    # print(heading_sp)

def get_link_angle_deg (orientation):
    orientation_q = orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # print(rudder.data)

    return rad2deg(yaw)
    
def listener():
    rospy.init_node('speed_control', anonymous=True)
    rospy.Subscriber("/eboat/mission_control/observations", Float32MultiArray, callback)
    rospy.Subscriber("/eboat/control_interface/speed_sp", Float32, callback_speed_sp)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()