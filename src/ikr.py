from __future__ import print_function
from fminbound import fminbound
# from scipy.optimize import fminbound
from numpy import *

DEG = pi/180.

SHOULDER_HEIGHT = 183.0
L1 = 210.0 
L2 = 221.5 
L3 =  23.7 # - 7.2
RHO = 30.0 
## http://www.mediafire.com/file/mnwe3ow14ixxcav/Mechanical_Specifications_%252811-09-2018%2529.pdf/file
bounds = array([[-175, 175],
                [-90, 36.7],
                [-80, 90],
                [-175, 175],
                [-100, 110],
                [-147.5, 147.5]]) * DEG
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

def rz_to_thetas(l1, l2, rho, r, delta_z, tol=.001, max_iter=400, full_output=False):
    '''
    l1 -- length of bone between sholder and elbow (210mm)
    l2 -- length of bone between elbow and wrist 41.5 + 180 = 221.5mm
    rho -- offset of elbow forarm pivot above elbow pivot. (30mm)
    r -- sqrt(x ** 2 + y ** 2) 2D radius of target in xy plane 
    delta_z -- height target is above the waist pivot
    tol -- mimimum distance between result and target
    max_iter -- maximum number of iterations.
    '''
    def get_rz(theta1, theta2):
        r_prime = -l1 * sin(theta1) + l2 * cos(theta1 + theta2)
        z_prime =  l1 * cos(theta1) + l2 * sin(theta1 + theta2)
        p_prime = array([r_prime, z_prime])
        offset = RHO * array([-sin(theta1 + theta2), cos(theta1 + theta2)])
        p = p_prime + offset
        return p
    theta1 = 0
    theta2 = 0

    def minme1(theta1, theta2): ### minimize theta1 with theta2 fixed
        r_prime, z_prime = get_rz(theta1, theta2)
        return (r_prime - r) ** 2 + (z_prime - delta_z) ** 2
    def minme2(theta2, theta1): ### minimize theta2 with theta1 fixed
        r_prime, z_prime = get_rz(theta1, theta2)
        return (r_prime - r) ** 2 + (z_prime - delta_z) ** 2
    
    for i in range(max_iter):
        theta1 = fminbound(minme1, bounds[1, 0], bounds[1, 1], args=(theta2,))
        theta2 = fminbound(minme2, bounds[2, 0], bounds[2, 1], args=(theta1,))
        rz = get_rz(theta1, theta2)
        err = linalg.norm(rz - [r, delta_z])
        if err < tol:
            break
    else:
        raise ValueError("Did not converge to (%s, %s)" % (r, delta_z))
    out = array([theta1, theta2]), err
    if not full_output:
        out = out[0]
    return out

def make_wrist_orthog(theta0, theta1, theta2, nhat, full_output=False):
    '''return theta3 so that wrist pivot is orthogonal to nhat
    '''
    R012 = dot(R_yaw(theta0), R_pitch(theta1 + theta2))
    def minme3(theta3):
        R = dot(R012, R_roll(theta3))
        ahat = R[:,1]
        out = abs(dot(ahat, nhat)) # + abs(theta3)*1e-8
        return out
    
    theta3 = fminbound(minme3, bounds[3, 0], bounds[3, 1])
    err = minme3(theta3)
    out = theta3, err
    if not full_output:
        out = out[0]
    return out
nhat = array([1, 2, 3])
nhat = nhat / linalg.norm(nhat)

def ik_4d(l0, l1, l2, rho, p_wrist, nhat):
    '''
    p -- position of wrist axis
    nhat -- direction of hand
    return angles 0 - 3
    '''
    px, py, pz = p_wrist
    ### waist theta0
    theta0 = arctan2(py, px)
    assert (bounds[0, 0] < theta0) and (theta0 < bounds[0, 1])

    ### 2 equation system for t1 and t2
    rxy = linalg.norm(p_wrist[:2])
    delta_z = p_wrist[2] - l0
    theta1, theta2 = rz_to_thetas(l1, l2, rho, rxy, delta_z)
    ### adjust forarm to put wrist axis ahat orthogonal to nhat
    theta3 = make_wrist_orthog(theta0, theta1, theta2, nhat)

    return array([theta0, theta1, theta2, theta3])

def ikr(l0, l1, l2, rho, p, nhat, roll):
    out = zeros(6)
    p_wrist = p - L3 * nhat
    print(format('      p', p))
    print(format('   nhat', nhat))
    print(format('p_wrist', p_wrist))
    
    
    theta0, theta1, theta2, theta3 = ik_4d(l0, l1, l2, rho, p_wrist, nhat)
    out[:4] = theta0, theta1, theta2, theta3

    ### rotate hand to align with nhat
    R0123 = dot(R_yaw(theta0), dot(R_pitch(theta1 + theta2), R_roll(theta3)))
    def minme4(theta4): ### wrist angle
        R = dot(R0123, R_pitch(theta4))
        return -dot(nhat, R[:,0]) + abs(theta4)/1e8
    theta4 = arange(-pi, pi, .01)
    out[4] = fminbound(minme4, bounds[4, 0], bounds[4, 1])
    #import pylab
    #pylab.plot(theta4 / DEG, [minme4(t4) for t4 in theta4])
    #pylab.plot(out[4] / DEG, minme4(out[4]), 'ro')
    #pylab.plot(bounds[4, 0] / DEG, minme4(bounds[4, 0]), 'bo')
    #pylab.plot(bounds[4, 1] / DEG, minme4(bounds[4, 1]), 'bo')
    #pylab.show()
    R = dot(R_pitch(out[4]), R0123)
    print('R:\n', R)
    print('minme4(out[4])', minme4(out[4]))
    
    out[5] = roll
    return out

fmt = '%10.4f'
def format(name, v, fmt=fmt):
    # print ('ikr.format:', name, v, fmt)
    return name + ' = [' + ', '.join([fmt % f for f in v]) + ']'

if __name__ == '__main__':
    ################################################################################
    ### TEST
    import robot

    # p = array([200, 0, .0])
    p = robot.HOME[:3]
    # 
    #p[0] = 100
    p[1] = -200
    #p[2] -= 400
    #p[1] = -200
    #p = [  221.5,     0.0000,   SHOULDER_HEIGHT + 30]
    #p = array([L2, 0, 183 + RHO])
    nhat = p
    nhat = array([1, 0, 1])
    nhat = nhat/linalg.norm(nhat)

    roll = 0
    theta = (ikr(SHOULDER_HEIGHT, L1, L2, RHO, p, nhat, roll))
    
    print(format('theta', theta / DEG, fmt), 'DEG')
    niryo = robot.robot
    niryo.move_joints(theta)
    p, rpy = niryo.get_arm_pose()
    print (format('    p', p, fmt))
    print (format('  rpy', rpy / DEG, fmt))


    from niryo_one_python_api.niryo_one_api import *
    import rospy
    rospy.init_node('niryo_one_example_python_api')

    n = NiryoOne()
    n.move_joints(theta)
