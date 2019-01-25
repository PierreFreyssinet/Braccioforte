import serial

import signal
import sys
import time

import CameraPublisher
import DemoArUco
import rospy
from std_msgs.msg import Int16MultiArray

stop = False

pub_aruco_n3 = rospy.Publisher('/jevois/ArUco/N3', Int16MultiArray, queue_size=1)
rospy.init_node('jevois_talker', anonymous=True)

def mean(l):
    out = sum(l) / float(len(l))
    return out

def publish(msg):
    if not rospy.is_shutdown():
        vals = msg
        if vals[-1].endswith('OK'):
            vals[-1] = vals[-1][:-2]
        while len(vals) >= 8:
            ## type, value, x, y, z, L, W, D
            t = vals[0]
            while t != 'N3' and t != '1N3' and len(vals) > 8:
                vals = vals[1:]
                t = vals[0]
            if t == 'N3':
                v = vals[1]
                v = int(v[1:])
                points = [v] + map(int, vals[2:])
                pub_aruco_n3.publish(data=points)
            vals = vals[8:]

def signal_handler(signal, frame):
    print('^C')
    global stop
    stop = True
    mode.close()
    sys.exit()


def on_error(err):
    global stop
    stop = True
    print str(err)


if __name__ == "__main__":
    serdev = '/dev/ttyACM0'
    signal.signal(signal.SIGINT, signal_handler)
    always_refresh = CameraPublisher.ALWAYS_REFRESH
    refresh_on_new = CameraPublisher.REFRESH_ON_NEW
    try:
        freq_callback = 0.05
        baudrate = 115200
        
        # mode_class = DiceCounter.DiceCounter
        # mode_class = QRcode.QRcode
        # mode_class = FaceTracking.FaceTracking
        mode_class = DemoArUco.DemoArUco
        
        mode = mode_class(serdev, callback=publish,
                          refresh_callback=refresh_on_new,
                          freq_callback=freq_callback,
                          baudrate=baudrate)
        mode.set_on_error(on_error)
        mode.start()
        t_end = time.time() + 10
        # while time.time() < t_end:
        while not stop:
            time.sleep(0.5)
            # print "get_value : " + str(mode.get_value())
    except serial.serialutil.SerialException as e:
        print str(e)
