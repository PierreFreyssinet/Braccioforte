import os
import serial
import subprocess
import threading

import time
import signal

import CameraPublisher
from guvcview_utils import set_guvcview_fps


class DiceCounter(threading.Thread):
    def on_error_received(self, e):
        self.on_error(e)

    def __init__(self, serdev, callback=None, refresh_callback=CameraPublisher.ALWAYS_REFRESH,
                 freq_callback=0.1, baudrate=115200, timeout=1):
        print "Init DiceCounter..."
        super(DiceCounter, self).__init__()
        self.on_error = None
        self.publisher = None
        self._pid = None
        self._gvcview = None
        self._baudrate = baudrate
        self._timeout = timeout
        self._serdev = serdev
        self._stop_event = threading.Event()

        try:
            self.publisher = CameraPublisher.CameraPublisher(
                serdev, callback, refresh_callback, freq_callback, baudrate, timeout)
            print "%s set_on_error %s" % (self.name, str(self.on_error_received))
            self.publisher.set_on_error(self.on_error_received)
        except serial.serialutil.SerialException as e:
            self.close()
            raise serial.SerialException(e)
        print "Init DiceCounter done"

    def start(self):
        print "Start DiceCounter..."
        try:
            video_num = 1
            set_guvcview_fps(7.5, video_num=video_num)
            self._gvcview = subprocess.Popen([
                'guvcview', '-d', '/dev/video%d' % video_num,
                '-g', 'none', '-a', 'none', '-f', 'YUYV',
                '-x', '640x480'], #, '-F', '7.5'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as e:
            if e.errno == os.errno.ENOENT:
                print "guvcview not found"
                print str(e)
                exit()
        self._pid = self._gvcview.pid
        self.publisher.start()
        self.publisher.read_cfg('DiceCounter.cfg')
        print "DiceCounter started"

    def close(self):
        print "Close DiceCounter..."
        if self._gvcview is not None:
            print "Closing Guvcview"
            os.kill(self._pid, signal.SIGTERM)
            self._gvcview.kill()
            self._gvcview = None
            print "Guvcview closed"
        self.publisher.close()
        self._stop_event.set()
        print "DiceCounter closed"

    def get_value(self):
        return self.publisher.get_value()

    def set_on_error(self, on_error):
        self.on_error = on_error
