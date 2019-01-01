#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy

def init():
    rospy.init_node('niryo_one_run_python_api_code')

pi = 3.14159
DEG = pi / 180.

starts = [-3.05433, -1.91986, -1.397485, -3.05433, -1.74533, -2.57436]
stops =  [ 3.05433, 0.640187,  1.570796,  3.05433,  1.91986,  2.57436]

limits = zip(starts, stops)

def limit_move(angles):
    angles = [x for x in angles] ### copy angles
    for i in range(6):
        if angles[i] < starts[i]:
            angles[i] = starts[i]
        if angles[i] > stops[i]:
            angles[i] = stops[i]
    return angles

def move_joints(niryo, angles, tries=5):
    angles = limit_move(angles)
    for k in range(tries - 1):
        try:
            niryo.move_joints(angles)
            break
        except NiryoOneException:
            pass
    else:
        niryo.move_joints(angles)

def move_pose(niryo, x, y, z, roll, pitch, yaw):
    niryo.move_pose(x, y, z, roll, pitch, yaw)

if __name__ == '__main__':
    init()
    niryo = NiryoOne()
    # niryo.calibrate_auto()
    niryo.activate_learning_mode(False)
    x = .2
    y = 0
    z = .5
    move_pose(niryo, x, y, z,  -0 * DEG, -90 * DEG, -90 * DEG)
    move_pose(niryo, x, y, z,   0 * DEG,  0 * DEG,  90 * DEG)
    move_pose(niryo, x, y, z,  -0 * DEG,  -0 * DEG, -90 * DEG)
    move_pose(niryo, x, y, z,  -0 * DEG,  -0 * DEG,  90 * DEG)
    move_pose(niryo, x, y, z, -90 * DEG,  -0 * DEG, -90 * DEG)
    move_pose(niryo, x, y, z,  90 * DEG,  -0 * DEG,  90 * DEG)
    zeros = (0, 0, 0, 0, 0, 0)
    for i in range(6):
        move_joints(niryo, zeros)

        p = [0, 0, 0, 0, 0, 0]
        p[i] = starts[i]
        move_joints(niryo, p)
        print p


        p[i] = stops[i]
        move_joints(niryo, p)
        print p

    move_joints(niryo, zeros)


