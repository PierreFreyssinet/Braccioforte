import rospy
from geometry_msgs.msg import Point
import serial
PORT = '/dev/ttyACM0'
s = serial.Serial(PORT, baudrate=115200, timeout=1)

def read():
    out = []
    while s.inWaiting() > 0:
        out.append(s.read(1))
        if out[-1] == '\n':
            break
    return ''.join(out)

s.read(10000)
s.write('streamoff\n'); print read ()
s.write('setpar serout USB\n'); print read()
s.write('setmapping2 YUYV 640 480 20 Jevois DemoArUco\n'); print read()
s.write('setpar serstyle Fine\n'); print read()
s.write('streamon\n'); print read()

pub_xyv = rospy.Publisher('/jevois/attention/xyv', Point, queue_size=10)
rospy.init_node('talker', anonymous=True)

def publish(x, y, v):
    if not rospy.is_shutdown():
        pub_xyv.publish(x, y, v)

def mean(l):
    out = sum(l) / float(len(l))
    return out

MAX_MSG = 1000
n_msg = 0
while n_msg < MAX_MSG:
    msg =read()
    if msg:
        vals = msg.strip().split()
        if len(vals) == 11:
            t, v, n, x0, y0, x1, y1, x2, y2, x3, y3 = vals
            v = int(v[1:])
            points = map(int, vals[3:])
            publish(mean(points[0::2]),
                    mean(points[1::2]), v)
            n_msg += 1
            print n_msg, '/', MAX_MSG, points
