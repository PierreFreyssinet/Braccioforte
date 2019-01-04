#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
from safe_move import limit_move, init, move_joints
print 'START'
init()
niryo = NiryoOne()
niryo.calibrate_auto()

niryo.activate_learning_mode(True)

poses = []
while 1:
    try:
        x = raw_input('...')
        if x == 'end':
            break
    except SyntaxError:
        pass
    pose = limit_move(niryo.get_joints())
    poses.append(pose)

niryo.activate_learning_mode(False)
print 'poses = ['
for pose in poses:
    print '      ', pose, ','
    move_joints(niryo, pose)
print ']'
niryo.activate_learning_mode(True)
