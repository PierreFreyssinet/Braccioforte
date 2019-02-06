from __future__ import print_function
'''
Coordinate frame library
'''
import numpy
from numpy import linalg, sin, cos, dot, cross, pi, array, arctan2, sqrt, abs
import pylab
from scipy.optimize import fminbound
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


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    # assert(isRotationMatrix(R))
     
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = arctan2(R[2,1] , R[2,2])
        y = arctan2(-R[2,0], sy)
        z = arctan2(R[1,0], R[0,0])
    else :
        x = arctan2(-R[1,2], R[1,1])
        y = arctan2(-R[2,0], sy)
        z = 0
 
    return array([x, y, z])

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

### inverse robot
I_HAND = CoordFrame('iHand', ORIGIN, [0, 0, 0], R_roll)
I_WRIST = CoordFrame('iWrist', I_HAND, [-23.7, 0, 5.5], R_pitch)
I_FORARM = CoordFrame('iForarm', I_WRIST, [-180, 0, 0], R_roll)
i_robot = Robot([I_HAND, I_WRIST, I_FORARM])

HOME = robot.move_joints([0, 0, 0, 0, 0, 0])

print('HOME', HOME)
print (HOME[:3], getR(HOME[3:]))

def foreward_kinematics(theta):
    return robot.move_joints(theta)
fk = foreward_kinematics

C_s = (1.) ** 2 # mm^2
C_a = (1. * DEG) ** 2
W_s = 1. / C_s
W_a = 1. / C_a 
W = numpy.diag([W_s] * 3 +  [W_a] * 3)

def inverse_kinematics_3x3(goal, tol=1e-6):
    theta = robot.get_theta()
    p = fk(theta)[:3]
    J = numpy.zeros((3, 3))
    goal = goal[:3]
    delta_p = goal - p
    if linalg.norm(delta_p) < tol:
        out = theta
    else:
        for i in range(10):
            delta_p = goal - p

            for j in range(3):
                h = 1 * DEG
                J[:,j] = ((fk(theta + I6[j] * h) - fk(theta - I6[j] * h))/
                          (2 * h))[:3]
            JTW = J.T @ W
            delta_theta = numpy.hstack([dot(linalg.pinv(dot(JTW, J)), dot(JTW, delta_p)),
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
    
def pre_ik(goal):
    ### return closest k positions
    resid = goal[numpy.newaxis] - POSITION
    Wr = (W @ resid.T).T
    rWr = numpy.sum(resid *  Wr, axis=1)
    rank = numpy.argsort(rWr)
    bests = rank[100:]
    out = THETA[bests]
    wssrs = rWr[bests]
    poses = POSITION[bests]
    print (wssrs)
    # pylab.close('all'); pylab.figure(); pylab.plot(wssrs); pylab.show();
    return out, poses, wssrs

def inverse_kinematics_old(goal, angle_tol=1 * DEG, position_tol=1):
    '''
    find angles to put hand at position goal
    '''
    theta0 = robot.get_theta()

    p0 = fk(theta0)
    J = numpy.zeros((6, 6))
    theta = theta0
    p = p0.copy()
    delta_p = goal - p
    fmt = '%10.4f'
    if (linalg.norm(delta_p[:3]) < angle_tol and
        linalg.norm(delta_p[3:]) < position_tol):
        out = theta0
    else:
        for i in range(40):
            delta_p = goal - p
            fmt = '%10.2f'
            for j in range(6):
                h = 1 * DEG
                J[:,j] = ((fk(theta + I6[j] * h) - fk(theta - I6[j] * h))/
                          (2 * h))
                #print(format(J[:,j], fmt))
            #print()
            JTW = J.T @ W
            # delta_theta = linalg.pinv(J.T @ J) @ J.T @ delta_p
            delta_theta = linalg.pinv(JTW @ J) @ JTW @ delta_p
            # print ('max(abs(delta_theta)) / DEG', max(abs(delta_theta)) / DEG)
            def minme(alpha):
                t = theta + alpha * delta_theta
                p = robot.move_joints(t)
                dp = p - goal
                n = dp @ W @ dp
                assert n >= 0
                return n

            x = numpy.arange(0, 1, .01)
            y = [minme(a) for a in x]
            alpha = fminbound(minme, -2, 2, xtol=.01)
            # pylab.clf()
            # pylab.plot(x, y)
            # pylab.plot(alpha, minme(alpha), 'ro')
            # pylab.show()
            theta = theta + alpha * delta_theta 
            theta = (theta + pi) % (2 * pi) - pi
            p = fk(theta)
            position_diff = linalg.norm(p[:3] - goal[:3])
            angle_diff    = linalg.norm(p[3:] - goal[3:])
            if position_diff < position_tol and angle_diff < angle_tol:
                break
        else:
            raise ValueError('Did not converge to \n%s\nfrom\n%s' % (goal, format(p0, fmt)))
    print ('p:!', format(p, fmt))
    print ('g:!', format(goal, fmt))
    print ('r:!', format(p - goal, fmt))
    print ('OK!', format(theta / DEG, fmt))
    print ()
    return theta

import ikpy
my_chain = ikpy.chain.Chain.from_urdf_file("my_mesh.urdf.xacro")
def inverse_kinematics(goal):
    target_vector = goal[:3] / 1000.
    target_frame = numpy.eye(4)
    target_frame[:3, :3] = getR(goal[3:])
    target_frame[:3,3] = target_vector
    out = my_chain.inverse_kinematics(target_frame)
    got = my_chain.forward_kinematics(out)
    print ('goal:', goal[:3])
    print (' err:', linalg.norm(goal[:3] - got[:3,3] * 1000.))
    return out[1:-1]
    
ik3 = inverse_kinematics_3x3
ik = inverse_kinematics_old
print (ik(HOME))
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

STEP =  10 * DEG
if False:
    arange = numpy.arange
    positions = []
    theta1 = numpy.linspace(-3.05433, 3.05433, 10)
    theta2 = numpy.linspace(-1.91986, 0.640187, 5)
    theta3 = numpy.linspace(-1.397485, pi/2, 5)
    theta4 = numpy.arange(-3.05433, -3.05433 + STEP/2, STEP)
    theta5 = numpy.arange(-1.74533, 1.91986 + STEP/2, STEP)
    theta6 = numpy.arange(-2.57436, 2.57436 + STEP/2, STEP)
    theta = numpy.vstack([x.ravel() for x in numpy.meshgrid(theta1, theta2, theta3, theta4, theta5, theta6)]).T
    position = numpy.empty((len(theta), 6))
    for i, t in enumerate(theta):
        if i % 1000 == 0:
            print (i, len(theta))
        position[i] = robot.move_joints([t[0], t[1], t[2], t[3], t[4], t[5]])
    numpy.save('position.npy', position)
    numpy.save('theta.npy', theta)
    print ('wrote position.npy, thetas.npy')
    
POSITION = numpy.load('position.npy')
THETA = numpy.load('theta.npy')

HOME = robot.move_joints([0, 0, 0, 0, 0, 0])

print ("********************************************************************************")
N = 21
xyz, rpy = robot.get_arm_pose()
r = linalg.norm(xyz[:2])
goals = numpy.zeros((N, 6)) + HOME
goals[:,0] = r * cos(numpy.arange(N) / N * pi / 2)
goals[:,1] = r * sin(numpy.arange(N) / N * pi / 2)
goals[:,2] = numpy.arange(N) * 500. / N
goals[:,3] = numpy.arange(N) * 90 * DEG / N
goals[:,4] = numpy.arange(N)/N * 90 * DEG - 25 * DEG
goals[:,5] = (numpy.arange(N)/N * 90 * DEG - 45 * DEG)
goals[0] = [  2.12349429e+02,   1.22600000e+02,   1.66666667e+02,   5.23598776e-01,   8.72664626e-02,  -2.61799388e-01]
fmt = '%10.4f'
for g in goals:
    print (format(g,fmt))
def border():
    ax.plot([-400, -400, -400, -400, 400, 400, 400, 400],
            [-400, -400, 400, 400, 400, 400, -400, -400],
            [0, 400, 0, 400, 0, 400, 0, 400], 'k,')
    
path = []

for goal in goals:
    try:
        bests, best_poses, wssrs = pre_ik(goal)
        print ('goal', goal)
        for i, (good, pose, wssr) in enumerate(zip(bests, best_poses, wssrs)):
            try:
                print (i, 'good', format(good, fmt), wssr)
                robot.move_joints(good)
                theta = ik(goal)
                robot.move_joints(theta)
                break
            except ValueError: ### try next best
                pass
        else:
            print(bests)
            print (goal)
            raise ValueError("Three tries... no find!")
        pts = robot.getPoints()
        #robot.move_joints(theta)
        # ax.plot(pts[:,0], pts[:,1], pts[:,2], '.-')
    except:
        raise
        break
