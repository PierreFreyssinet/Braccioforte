#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
from niryo_one_python_api.niryo_one_api import *
rospy.init_node('niryo_one_example_python_api')
from numpy import sin, cos, dot, array, transpose, pi

def rotation(axis, amount):
    if axis == 2:
        R = array([[cos(amount), -sin(amount), 0],
                   [sin(amount),  cos(amount), 0],
                   [0, 0, 1]])
    elif axis == 1:
        R = array([[cos(amount), 0, -sin(amount)],
                   [0, 1, 0],
                   [sin(amount),  0, cos(amount)]])
    elif axis == 0:
        R = array([[1, 0, 0],
                   [0, cos(amount), -sin(amount)],
                   [0, sin(amount), cos(amount)]])
    return R

def get_camera_pos():
    h1 = .19 ## high of axis 2 from ground
    h2 = .21 ## length of upper arm
    h3 = .08 ## offset of camer from axis 3
    
    joints = niryo.get_joints()
    theta = joints[0]
    rho = joints[1]
    psi = joints[2]

    R1 = rotation(2, theta)
    R2 = rotation(1, rho)
    orient_elbow = dot(R1, R2)
    R3 = rotation(1, psi)

    rot = ([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) # camera xyz to real
    orient = dot(R1, dot(R2, dot(R3, rot)))

    p1 = array([0, 0, h1])
    p2 = p1 + dot(orient_elbow, [0, 0, h2])
    p3 = dot(orient, [0, 0, h3])
    pos = p2 + p3
    return pos, orient, joints

DEG = pi / 180

start = [30*DEG, 0, -40 * DEG, 0, -10 * DEG, 0]
start = [0]  * 6
# start[1] = -pi/2.
niryo = NiryoOne()
# niryo.calibrate_auto()
niryo.instance.set_arm_max_velocity(100)

niryo.move_joints(start)
#niryo.shift_pose(0, .05) ### X
#niryo.shift_pose(1, .05) ### Y
#niryo.shift_pose(2, .05) ### Z
# niryo.shift_pose(4, 5 * DEG) ### pitch
for i in []: ## [0, 2]
    pos = start[:]
    for j in range(-10, 10):
        pos[i] = start[i] + j * DEG
        try:
            niryo.move_joints(pos)
        except NiryoOneEcxeption:
            pass

gain_y = 1. * DEG / 4
gain_x = 1. * DEG / 4
count = 0
pos = [v for v in start]
MAX_WRIST = 0

def callback(data):
    global count
    count += 1
    
    if data.data[0] == 42:
        move = False
        STEP = .5 * DEG
        TOL = 10
        # rospy.loginfo(rospy.get_caller_id() + "\n%s" + str(data))
        vals = data.data
        value = vals[0]
        offset_cam = array(vals[1:4]) / 1000.
        pos_cam, orient_cam, joints = get_camera_pos()
        pos_abs = pos_cam + dot(orient_cam, offset_cam)
        x, y, z = pos_abs
        print x, y, z
        try:
            niryo.move_pose(x-.1, y, z, 0, 40 * DEG, 0)
        except:
            pass
        niryo.move_joints(start)
        return
        # niryo.move_pose(.25, 0, .4, 0, 0, 0)
        if abs(y) > TOL:
            pos[2] -= y * gain_y
            move = True
        if abs(x) > TOL:
            pos[0] -=  x * gain_x
            move = True
        if move:
            pose = niryo.get_pose()
            pose[0] += x
            pose[1]
            niryo.move_pose(pos)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.Subscriber("jevois/ArUco/N3", Int16MultiArray, callback, queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()
