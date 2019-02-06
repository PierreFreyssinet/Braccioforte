from niryo_one_python_api.niryo_one_api import *
from math import *
import rospy
import time
from numpy import *
rospy.init_node('niryo_one_example_python_api')
niryo = NiryoOne()
try:
    niryo.calibrate_auto()
except NiryoOneException:
    pass
niryo.change_tool(TOOL_GRIPPER_2_ID)
niryo.instance.set_arm_max_velocity(100)

DEG = pi / 180.
M = 1
MM = .001 * M
INCH = 25.4 * MM
z = .38
x = .24
y = .0
gripper_length = .100

def move_gripper(x, y, z, roll, pitch, yaw, n_try=2, xtra=0):
    out = True
    for i in range(n_try):
        try:
            niryo.move_pose(x - cos(pitch) * cos(yaw) * (gripper_length + xtra),
                            y - cos(pitch) *  sin(yaw) * (gripper_length + xtra),
                            z + sin(pitch) * (gripper_length + xtra), 0, pitch, yaw)
            break
        except NiryoOneException, e:
            pass
    else:
        print e
        out = False
    return out
home = [-0.05441108925805003, 0.6358772498844905, -0.40142572795869574,
        -0.07065092812073047, -0.20120155616990631, 0.035430183815484885]
max_pitch = 55 * DEG
niryo.close_gripper(TOOL_GRIPPER_2_ID, 1000)
if False:
    for yaw_deg in range(-24, 25, 8):
        yaw = yaw_deg * DEG
        for pitch_deg in range(-60, 61, 20):
            pitch = pitch_deg * DEG
            print pitch_deg, yaw_deg
            move_gripper(x, y, z, 0, pitch, yaw, xtra=1*INCH)
            #niryo.open_gripper(TOOL_GRIPPER_2_ID, 1000)
            move_gripper(x, y, z, 0, pitch, yaw, xtra=0)
            #niryo.close_gripper(TOOL_GRIPPER_2_ID, 1000)
            #niryo.open_gripper(TOOL_GRIPPER_2_ID, 1000)
            move_gripper(x, y, z, 0, pitch, yaw, xtra=1*INCH)
            # niryo.move_joints(home)
if False:
    pitch = 0
    for yaw_deg in range(-25, 28, 25):
        print yaw_deg
        yaw = yaw_deg * DEG

        niryo.move_pose(x - cos(yaw) * gripper_length,
                        y - sin(yaw) * gripper_length,
                        z, 0, 0, yaw)
for pitch in arange(max_pitch, -max_pitch, -1 * DEG):
    print pitch
    niryo.move_pose(x - gripper_length * cos(pitch), 0,
                    z + gripper_length * sin(pitch), 0, pitch, 0)
niryo.move_pose(x - gripper_length * cos(-max_pitch), 0,
                z + gripper_length * sin(-max_pitch), 0, -max_pitch, 0)
niryo.activate_learning_mode(True)

