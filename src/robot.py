from __future__ import print_function
'''
Coordinate frame library
'''
import numpy
from numpy import linalg, sin, cos, dot, cross, pi, array, arctan2, sqrt, abs
DEG = pi/180
I3 = numpy.eye(3)
I6 = numpy.eye(6)

def R_roll(theta):
    return array([[1, 0, 0],
                 [0, cos(theta), -sin(theta)],
                 [0, sin(theta),  cos(theta)]])
def R_pitch(theta):
    return array([[cos(theta), 0, -sin(theta)],
                  [0, 1., 0],
                  [sin(theta), 0, cos(theta)]])
def R_yaw(theta):
    return array([[cos(theta), -sin(theta), 0],
                  [sin(theta),  cos(theta), 0],
                  [0, 0, 1]])

def Null(theta):
    return I3

class CoordFrame:
    def __init__(self, name, ref, offset, R, angle=0):
        '''
        represent a new coord from offset from ref_point by
        rotation R
        ref -- CoordFrame (or none for standard basis reference)
        offset -- 3-vec
        R -- function (R_roll, R_pitch or R_yaw)
        '''
        self.name = name
        self.ref = ref
        self.offset = offset
        self.R = R
        self.angle = angle

    def getBasis(self):
        out = self.R(self.angle)
        ref_basis = self.ref.getBasis()
        out = dot(out, ref_basis)
        return out
    
    def getPoint(self):
        '''
        compute offset point location by rotating offset about axis by
        theta and adding ref_point
        '''
        p = dot(self.ref.getBasis(), self.offset) + self.ref.getPoint()
        return p
    def setAngle(self, angle):
        self.angle = angle
    def __repr__(self):
        return '\nCoordFrame:%s\n%s,\n%s' % (self.name,
                                           self.getBasis(), self.getPoint())
    
class Origin(CoordFrame):
    angle = 0.
    name = 'ORIGIN'
    def __init__(self):
        pass
    def getBasis(self):
        return I3
    def getPoint(self):
        return array([0, 0, 0.])
    def setAngle(self, angle):
        raise ValueError('Cannot set angle of Origin!')
    def __repr__(self):
        return 'ORIGIN'

def getRPY(R):
    roll = arctan2(R[2, 1], R[2, 2])
    pitch = arctan2(R[2, 0], sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = arctan2(R[1, 0], R[0, 0])
    return array([roll, pitch, yaw])

def getR(rpy):
    roll, pitch, yaw = rpy
    return dot(R_yaw(yaw), dot(R_pitch(pitch), R_roll(roll)))

rpy = .1, .2, -.3
R = getR(rpy)
assert abs(linalg.norm(getRPY(R) - rpy)) < 1e-8

class Robot:
    def __init__(self, frames):
        self.frames = frames
    def move_joints(self, angles):
        for i in range(1, len(self.frames) - 1): ## skip origin and tool
            frame = self.frames[i]
            frame.setAngle(angles[i-1])
        return numpy.hstack(self.get_arm_pose())
    
    def getPoints(self):
        return array([f.getPoint() for f in self.frames])

    def __repr__(self):
        return '%s' % self.getPoints()

    def get_arm_pose(self):
        p = self.frames[-1].getPoint()
        rpy = getRPY(self.frames[-1].getBasis())
        return (p, rpy)
    def get_theta(self):
        return array([frame.angle for frame in self.frames])[1:-1] ## skip first and last

ORIGIN = Origin()
WAIST = CoordFrame('Waist', ORIGIN, [0, 0, 103], R_yaw)
SHOULDER = CoordFrame('Shoulder', WAIST, [0, 0, 80], R_pitch)
ELBOW = CoordFrame('Elbow', SHOULDER, [0, 0, 210], R_pitch)
FORARM = CoordFrame('Forarm', ELBOW, [41.5, 0, 30], R_roll)
WRIST = CoordFrame('Wrist', FORARM, [180, 0, 0], R_pitch)
HAND = CoordFrame('Hand', WRIST, [23.7, 0, -5.5], R_roll)
TOOL = CoordFrame('Tool', HAND, [0, 0, 0], Null)

robot = Robot([ORIGIN, WAIST, SHOULDER, ELBOW, FORARM, WRIST, HAND, TOOL])
HOME = robot.move_joints([0, 0, 0, 0, 0, 0])
