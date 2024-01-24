#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from gazebo_msgs.msg import ModelStates, LinkStates
from simple_pid import PID
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import rad2deg

pub = rospy.Publisher('eboat/control_interface/rudder', Float32, queue_size=10)
pid_rudder = PID(1, 0.1, 0.05)

def callback(data):
    # pega a posicao atual e a posicao que se deseja alcancar e calcula o angulo entre elas;
    # tenta manter na reta que conecta os dois pontos;
    
    # pid_rudder.setpoint = ;
    # current_rudder = ;

    # tmp = Float32()
    # tmp.data = 3
    # pub.publish(tmp)
    # rospy.loginfo(data)
    rudder_link_name = 'eboat::rudder_link'
    rudder_link_index = data.name.index(rudder_link_name)
    rudder_angle_current = get_link_angle_deg(data.pose[rudder_link_index].orientation)
    print(rudder_angle_current)
    # print(data.pose[rudder_link_index].orientation)

def get_link_angle_deg (orientation):
    orientation_q = orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return rad2deg(yaw)
    
def listener():
    rospy.init_node('pid_control', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()