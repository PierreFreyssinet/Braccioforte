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
        return 'CoordFrame:%s\n%s,\n%s' % (self.name,
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

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def getRPY(R):
    roll = arctan2(R[2, 1], R[2, 2])
    pitch = arctan2(R[2, 0], sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = arctan2(R[1, 0], R[0, 0])
    return array([roll, pitch, yaw])

def getR(ryp):
    roll, pitch, yaw = rpy
    return dot(R_yaw(yaw), dot(R_pitch(pitch), R_roll(roll)))

rpy = -.1, -.2, -.3
R = getR(rpy)
assert abs(linalg.norm(getRPY(R) - rpy)) < 1e-8

class Robot:
    def __init__(self, frames):
        self.frames = frames
    def move_joints(self, angles):
        for i in range(1, len(self.frames)): ## skip origin
            frame = self.frames[i]
            frame.setAngle(angles[i-1])
    def getPoints(self):
        return array([f.getPoint() for f in self.frames])

    def __repr__(self):
        return '%s' % self.getPoints()

    def get_arm_pose(self):
        p = self.frames[-1].getPoint()
        rpy = getRPY(self.frames[-1].getBasis())
        return (p, rpy)
    def get_theta(self):
        return array([frame.angle for frame in self.frames])[1:]

ORIGIN = Origin()
WAIST = CoordFrame('Waist', ORIGIN, [0, 0, 103], R_yaw)
SHOULDER = CoordFrame('Shoulder', WAIST, [0, 0, 80], R_pitch)
ELBOW = CoordFrame('Elbow', SHOULDER, [0, 0, 210], R_pitch)
FORARM = CoordFrame('Forarm', ELBOW, [41.5, 0, 30], R_roll)
WRIST = CoordFrame('Wrist', FORARM, [180, 0, 0], R_pitch)
HAND = CoordFrame('Hand', WRIST, [23.7, 0, -5.5], R_roll)
robot = Robot([ORIGIN, WAIST, SHOULDER, ELBOW, FORARM, WRIST, HAND])

def foreward_kinematics(theta):
    robot.move_joints(theta)
    p, q = robot.get_arm_pose()
    return numpy.hstack([p, q])
fk = foreward_kinematics

def inverse_kinematics_3x3(goal, tol=1e-6):
    goal = goal[:3]
    theta = robot.get_theta()
    p = fk(theta)[:3]
    J = numpy.zeros((3, 3))

    delta_p = goal - p
    if linalg.norm(delta_p) < tol:
        out = theta
    else:
        for i in range(20):
            delta_p = goal - p

            for j in range(3):
                h = 1 * DEG
                J[:,j] = ((fk(theta + I6[j] * h) - fk(theta - I6[j] * h))/
                          (2 * h))[:3]
            delta_theta = numpy.hstack([dot(linalg.pinv(dot(J.T, J)), dot(J.T, delta_p)),
                                        numpy.zeros(3)])
            theta = theta + delta_theta
            p = fk(theta)[:3]
            d = linalg.norm(p - goal)
            if d < tol:
                break
        else:
            raise ValueError('Did not converge to %s' % goal)
    # print 'OK3!', goal[:3], theta / DEG, 'DEG'
    return theta

def format(arr, fmt):
    return ', '.join([fmt % v for v in arr])
    
def inverse_kinematics(goal, tol=1 * DEG):
    '''
    find angles to put hand at position goal
    '''
    theta0 = robot.get_theta()
    if True:
        theta0 = numpy.hstack([inverse_kinematics_3x3(goal)[:3],
                               theta0[3:]])
        theta0 = (theta0 + pi) % (2 * pi) - pi
    p0 = fk(theta0)
    J = numpy.zeros((6, 6))
    theta = theta0
    p = p0.copy()
    delta_p = goal - p
    if linalg.norm(delta_p) < tol:
        out = theta0
    else:
        space_w = 1.
        angle_w = (100 * DEG) ** 2
        W = numpy.diag([space_w] * 3 +  [angle_w] * 3)
        for i in range(200):
            delta_p = goal - p

            for j in range(6):
                h = 1 * DEG
                J[:,j] = ((fk(theta + I6[j] * h) - fk(theta - I6[j] * h))/
                          (2 * h))
            JTW = dot(J.T, W)
            #delta_theta = dot(linalg.pinv(dot(J.T, J)), dot(J.T, delta_p))
            delta_theta = dot(linalg.pinv(dot(JTW, J)), dot(JTW, delta_p))
            theta = theta + delta_theta
            theta = (theta + pi) % (2 * pi) - pi
            p = fk(theta)
            d = linalg.norm(p - goal)
            if d < tol:
                break
        else:
            raise ValueError('Did not converge to %s' % goal)
    fmt = '%10.4f'
    print 'p:!', format(p, fmt)
    print 'g:!', format(goal, fmt)
    print 'r:!', format(p - goal, fmt)
    print 'OK!', format(theta / DEG, fmt)
    print
    return theta

SPACE_W = 1. ** 2
ANGLE_W = 100/DEG ** 2
W = array([SPACE_W] * 3 +  [ANGLE_W] * 3)
def pre_ik(goal):
    res = goal[numpy.newaxis] - POSITION
    wssr = numpy.sum(res ** 2 * W, axis=1)
    i = numpy.argmin(wssr)
    out = numpy.hstack([THETA[i], 0])
    return out

ik3 = inverse_kinematics_3x3
ik = inverse_kinematics

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

if False:
    arange = numpy.arange
    positions = []
    theta1 = arange(0, 176, 10) * DEG
    theta2 = arange(-90, 41, 10) * DEG
    theta3 = arange(-80, 91, 10) * DEG
    theta4 = arange(-175, 176, 20) * DEG
    theta5 = arange(-100, 111, 20) * DEG
    theta = numpy.vstack([x.ravel() for x in numpy.meshgrid(theta1, theta2, theta3, theta4, theta5)]).T
    position = numpy.empty((len(theta), 6))
    for i, t in enumerate(theta):
        if i % 1000 == 0:
            print i, len(theta)
        robot.move_joints([t[0], t[1], t[2], t[3], t[4], 0])
        pts = robot.getPoints()
        position[i] = numpy.hstack(robot.get_arm_pose())
    numpy.save('position.npy', position)
    numpy.save('theta.npy', theta)
    print 'wrote position.npy, thetas.npy'
    here
else:
    POSITION = numpy.load('position.npy')
    THETA = numpy.load('theta.npy')

N = 19
angle = (numpy.arange(N)) * 60 * DEG / N
goals = numpy.zeros((N, 6)) + numpy.hstack(robot.get_arm_pose())[numpy.newaxis]
r = linalg.norm(robot.get_arm_pose()[0][:2])
goals[:,0] = r * cos(angle)
goals[:,1] = r * sin(angle)
goals[:,5] = angle
# goals[:,2] += numpy.arange(10) 
# goals[:,3] += numpy.arange(10) * 10 * DEG
# goals[:,4] += numpy.arange(10) * 1 * DEG
# goals[:,5] += numpy.arange(10) * 1 * DEG
# goals[:,1] += numpy.arange(100) -50

def border():
    ax.plot([-400, -400, -400, -400, 400, 400, 400, 400],
            [-400, -400, 400, 400, 400, 400, -400, -400],
            [0, 400, 0, 400, 0, 400, 0, 400], 'k,')
    
path = []

for goal in goals:
    try:
        xgoal = goal.copy()
        xgoal[0] = abs(goal[0])
        pre_theta = pre_ik(xgoal)
        # pre_theta[0] = sign(goal[0]) * pre_theta[0]
        robot.move_joints(pre_theta)
        print 'goal', goal
        print 'pre_theta', pre_theta
        print 'pose', robot.get_arm_pose()
        print
        theta = ik(goal)
        pts = robot.getPoints()
        ax.plot(pts[:,0], pts[:,1], pts[:,2], '.-')
    except:
        raise
        break
border()
plt.show()
here
    


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
pts = robot.getPoints()
ax.plot(pts[:,0], pts[:,1], pts[:,2], 'k.-')
print 'pts[-1]', pts[-1]
if True:
    border()
robot.move_joints([0.0004248837065551556, -1.2027942074430324, -0.6845908157736582, -0.00285437370594186, 0.3164445089581473, -1.5666838848979696])
pts = robot.getPoints()
ax.plot(pts[:,0], pts[:,1], pts[:,2], '.-')
plt.show()
herex
for i in range(0, 91, 10):
    robot.move_joints([0, i * DEG, 0])
    pts = robot.getPoints()
    ax.plot(pts[:,0], pts[:,1], pts[:,2], 'r.-')
plt.axis('equal')
plt.show()

print
