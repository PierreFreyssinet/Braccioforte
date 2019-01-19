#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from niryo_one_python_api.niryo_one_api import *
rospy.init_node('niryo_one_example_python_api')

pi = 3.14159
DEG = pi / 180

start = [30*DEG, 0, -40 * DEG, 0, -10 * DEG, 0]
niryo = NiryoOne()
niryo.calibrate_auto()
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

gain_wrist = 1. * DEG/25
gain_elbow = 1. * DEG / 250.
gain_x = 1. * DEG/40
count = 0
pos = [v for v in start]
MAX_WRIST = 0
def callback(data):
        global count
        count += 1
        move = False
        STEP = .5 * DEG
        TOL = 10
        rospy.loginfo(rospy.get_caller_id() + "X: %f, Y: %f", data.x, data.y)
        if abs(data.y) > TOL:
                wrist_angle = data.y * gain_wrist
                if pos[4] < MAX_WRIST + wrist_angle:
                        pos[4] -=  wrist_angle
                        move = True
                else:
                        pos[4] -= MAX_WRIST
                        pos[2] -= data.y * gain_elbow
        if abs(data.x) > TOL:
                pos[0] -=  data.x * gain_x
                move = True
        if move:
                niryo.move_joints(pos)

        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.Subscriber("jevois/attention/xyv", Point, callback, queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    listener()
