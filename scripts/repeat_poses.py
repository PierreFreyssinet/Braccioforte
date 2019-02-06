from niryo_one_python_api.niryo_one_api import *
import rospy
import time
rospy.init_node('niryo_one_example_python_api')

home = [0, 0, -1.345, -0.009000000000000001, 0.009000000000000001, -0.005]
n = NiryoOne()
pi = 3.14159
deg = pi / 180.

n.calibrate_auto()
n.instance.set_arm_max_velocity(100)
poses = [
    [0, 0, 0, 0, 0, 0],
    ]
for pos in poses:
    n.move_joints(pos)

