from niryo_one_python_api.niryo_one_api import *
import rospy
import time
rospy.init_node('niryo_one_example_python_api')

home = [0, 0, -1.345, -0.009000000000000001, 0.009000000000000001, -0.005]
# home = [0] * 6
n = NiryoOne()

pi = 3.14159
deg = pi / 180.
z = .4
x = .24
gripper_length = .115
gripper_length = 0
n.instance.set_arm_max_velocity(50)
# n.calibrate_manual()
# n.calibrate_auto()
n.move_joints(home)
# moves = [x, y, z, r, p, yaw]
moves = [[.15, 0],
         [-.1, 0],
         ]
for v, a in moves:
    print a, v, n.shift_pose(a, v);
    
n.move_joints(home)
n.move_pose(x - gripper_length, 0, z, 0, 0, 0)
n.move_pose(x, 0, z + gripper_length, 0, pi/2, 0)
n.move_pose(x - gripper_length, 0, z, 0, 0, 0)
n.move_pose(x, -gripper_length, z, 0, -pi/2, 0)
n.move_pose(x, + gripper_length, z, 0, 0, pi/2)
n.move_pose(x, 0, z, 0, 0, -pi/2)
n.activate_learning_mode(True)
here

n.move_joints([0, -0.321, -0.163, 0.029, 0.464, -0.015])
n.move_joints([0.515, -0.836, -0.659, 0.501, 1.493, -0.035])
n.move_joints([-1.179, -0.328, 0.507, -1.805, 1.218, -1.463])
n.move_joints([-1.179, 0.064, -0.6960000000000001, 2.2640000000000002, 1.473, 1.832])
n.move_joints([1.836, -0.502, 0.97, -2.275, -1.227, -1.478])
n.move_joints([0, 0, 0, 0, 0, 0])
n.move_joints([0, 0, 0, 2.209, 0.005, -2.222])
n.move_joints([0, 0, -1.345, -0.009000000000000001, 0.009000000000000001, -0.005])
n.activate_learning_mode(True)
