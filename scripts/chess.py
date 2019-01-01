from niryo_one_python_api.niryo_one_api import *
import rospy
import time
rospy.init_node('niryo_one_example_python_api')
import random

n = NiryoOne()
pi = 3.14159
mm = .001
inch = 25.4 * mm
z = .0
W = 1.25 * inch
W = 250 * mm / 8
dz = 5 * inch
xoff = .1
n.change_tool(TOOL_GRIPPER_2_ID)
n.open_gripper(TOOL_GRIPPER_2_ID, 1000)
n.close_gripper(TOOL_GRIPPER_2_ID, 1000)
def goto(sq):
    col = ord(sq[0]) - ord('A')
    row = ord(sq[1]) - ord('1')
    pitch = pi/2
    print sq
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z + dz, 0, pitch, 0)
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z, 0, pitch, 0)
    n.move_pose(xoff + col * W, row * W - 3.5 * W, z + dz, 0, pitch, 0)

def move(start, stop):
    goto(start)
    goto(stop)

goto('H8')
goto('H1')
goto('A1')
goto('A8')

while 1:
    row = random.choice('12345678')
    col = random.choice('ABCDEFGH')
    goto(col + row)

