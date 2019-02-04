from niryo_one_python_api.niryo_one_api import *
import rospy
import time
from numpy import pi
rospy.init_node('niryo_one_example_python_api')

home = [0, 0, -1.345, -0.009000000000000001, 0.009000000000000001, -0.005]
# home = [0] * 6
n = NiryoOne()

n.move_pose(.2, 0, 0, 0, pi/2, 0)

