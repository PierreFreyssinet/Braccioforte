from niryo_one_python_api.niryo_one_api import *
import rospy
import time
rospy.init_node('niryo_one_example_python_api')
import random

n = NiryoOne()
n.calibrate_auto()
pi = 3.14159
mm = .001
inch = 25.4 * mm
z = .15
W = 1.25 * inch
W = 250 * mm / 8
dz = 5 * inch
xoff = .1
n.change_tool(TOOL_GRIPPER_2_ID)

def move(frm, _to):
    col = ord(frm[0]) - ord('A')
    row = ord(frm[1]) - ord('1')
    pitch = pi/2
    print frm, _to
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z + dz, 0, pitch, 0)
    n.open_gripper(TOOL_GRIPPER_2_ID, 1000)
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z, 0, pitch, 0)
    n.close_gripper(TOOL_GRIPPER_2_ID, 1000)
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z + dz, 0, pitch, 0)
    col = ord(_to[0]) - ord('A')
    row = ord(_to[1]) - ord('1')
    pitch = pi/2

    n.move_pose(xoff + col * W, row * W - 3.5 * W, z + dz, 0, pitch, 0)
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z, 0, pitch, 0)
    n.open_gripper(TOOL_GRIPPER_2_ID, 1000)
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z + dz, 0, pitch, 0)

for l in open('moves.txt').readlines():
    frm_row = random.choice('12345678')
    frm_col = random.choice('ABCDEFGH')
    #frm = frm_col + frm_row
    frm = l[:2]
    to_row = random.choice('12345678')
    to_col = random.choice('ABCDEFGH')
    #_to = to_col + to_row
    _to = l[2:4]
    move(frm, _to)

