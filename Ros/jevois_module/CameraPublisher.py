import serial
from threading import Thread

import time

import JevoisSerial

REFRESH_ON_NEW = 2
ALWAYS_REFRESH = 1


class CameraPublisher:
    def __init__(self, serdev, callback=None, refresh_callback=ALWAYS_REFRESH,
                 freq_callback=0.1, baudrate=115200, timeout=1):
        '''try:
            self._ser = serial.Serial(serdev, baudrate=baudrate, timeout=timeout)
        except serial.SerialException as e:
            print str(e)
            raise serial.SerialException(e)'''
        print "Init CameraPublisher"
        self._ser = None
        self._value = None
        self._stop = False
        self._prev_value = None
        self.on_error = None
        self._refresh_callback = refresh_callback
        self._callback = callback
        self._freq_callback = freq_callback
        try:
            self.jevois_serial = JevoisSerial.JevoisSerial(serdev, baudrate, timeout)
        except serial.SerialException as e:
            self.close()
            raise serial.SerialException(e)

        if self._callback is not None:
            self._callback_get = Thread(target=self.callback_get_value)
        else:
            self._callback_get = None
        self.publisher = Thread(target=self.publish_value)
        print "Init CameraPublisher Done"

    def start(self):
        print "Starting CameraPublisher..."
        self._ser = self.jevois_serial.start()
        if self._callback is not None:
            self._callback_get.start()
        self.publisher.start()
        print "CameraPublisher started"

    def get_value(self):
        return self._value

    def publish_value(self):
        while not self._stop:
            out = ''
            try:
                while self._ser.inWaiting() > 0:
                    out += self._ser.read(1)
            except IOError as e:
                print "Error : " + str(e)
                print self.on_error
                if self.on_error is not None:
                    self.on_error(e)
                self.close()
            if out != '':
                out = out.replace('\n', '').replace('\r', '')
                # print str(out)
                out = out.split(' ')
                if len(out) > 1:
                    '''Filter the output depending on the id received'''
                    if out[1] == 'QR-Code':
                        self._value = ('QR-Code', out[-1])
                    elif out[1] == 'sm':
                        self._value = ('Tracker', (out[2], out[3], out[4], out[5]))
                    elif out[0] == 'PIPS':
                        self._value = ('DiceCount', out[1])
                    else:
                        self._value = out

    def callback_get_value(self):
        while not self._stop:
            if self._value is not None:
                if self._refresh_callback == ALWAYS_REFRESH:
                    self._callback(self._value)
                elif self._refresh_callback == REFRESH_ON_NEW:
                    if self._prev_value != self._value:
                        self._callback(self._value)
                        self._prev_value = self._value
            time.sleep(self._freq_callback)

    def close(self):
        print "Close CameraPublisher"
        self._stop = True

    def set_on_error(self, on_error):
        self.on_error = on_error

    def read_cfg(self, cfg):
        self.jevois_serial.read_cfg(cfg)
