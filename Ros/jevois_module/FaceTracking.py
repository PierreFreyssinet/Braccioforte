import os
import serial
import subprocess
import threading

import signal
import time

import CameraPublisher
import JevoisSerial
from guvcview_utils import set_guvcview_fps


class FaceTracking(threading.Thread):
    def on_error_received(self, e):
        self.on_error(e)

    def __init__(self, serdev, callback=None, refresh_callback=CameraPublisher.ALWAYS_REFRESH,
                 freq_callback=0.1, baudrate=115200, timeout=1):
        print "Init Tracker..."
        super(FaceTracking, self).__init__()
        self._gvcview = None
        self.on_error = None
        self._pid = None
        self._baudrate = baudrate
        self._timeout = timeout
        self._serdev = serdev
        self._stop_event = threading.Event()

        try:
            self.publisher = CameraPublisher.CameraPublisher(
                serdev, callback, refresh_callback, freq_callback, baudrate, timeout)
            # print "TRACKER set_on_error " + str(self.on_error_received)
            self.publisher.set_on_error(self.on_error_received)
        except serial.serialutil.SerialException as e:
            self.close()
            raise serial.SerialException(e)
        print "Init Tracker done"

    def start(self):
        print "Start Tracker..."
        try:
            video_num = 1
            set_guvcview_fps(50, video_num)
            self._gvcview = subprocess.Popen([
                'guvcview', '-d', '/dev/video%d' % video_num,
                '-g', 'none', '-a', 'none', '-f', 'YUYV',
                '-x', '640x312'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as e:
            if e.errno == os.errno.ENOENT:
                print "gucvview not fount=d"
                print str(e)
                exit()
        time.sleep(2)
        self._pid = self._gvcview.pid
        self.publisher.start()
        self.publisher.read_cfg('FaceTracking.cfg')
        print "Tracker started"

    def close(self):
        print "Close Tracker..."
        if self._gvcview is not None:
            print "Closing Guvcview"
            print self._pid
            os.kill(self._pid, signal.SIGTERM)
            self._gvcview.kill()
            self._gvcview = None
            print "Guvcview closed"
        self._stop_event.set()
        self.publisher.close()
        print "Tracker closed"

    def get_value(self):
        return self.publisher.get_value()

    def set_on_error(self, on_error):
        self.on_error = on_error
