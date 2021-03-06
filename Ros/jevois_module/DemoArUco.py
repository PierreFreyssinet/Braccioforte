import os
import serial
import subprocess
import threading

import time
import signal

import CameraPublisher

class DemoArUco(threading.Thread):
    def on_error_received(self, e):
        self.on_error(e)

    def __init__(self, serdev, callback=None, refresh_callback=CameraPublisher.ALWAYS_REFRESH,
                 freq_callback=0.1, baudrate=115200, timeout=1):
        super(DemoArUco, self).__init__()
        self.name = self.__class__.__name__
        print "Init %s..." % self.name
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
            print "%s set_on_error %s" % (self.name,
                                          str(self.on_error_received))
            self.publisher.set_on_error(self.on_error_received)
        except serial.serialutil.SerialException as e:
            self.close()
            raise serial.SerialException(e)
        print "Init %s done" % self.name

    def start(self):
        print "Start %s ..." % self.name
        self.publisher.start()
        self.publisher.read_cfg('%s.cfg' % self.name)
        print "%s started" % self.name

    def close(self):
        print "Close %s..." % self.name
        if self._gvcview is not None:
            print "Closing Guvcview"
            os.kill(self._pid, signal.SIGTERM)
            self._gvcview.kill()
            self._gvcview = None
            print "Guvcview closed"
        self.publisher.close()
        self._stop_event.set()
        print "%s closed" % self.name

    def get_value(self):
        return self.publisher.get_value()

    def set_on_error(self, on_error):
        self.on_error = on_error
