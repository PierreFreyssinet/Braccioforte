import serial
import threading
import subprocess

import CameraPublisher
from guvcview_utils import set_guvcview_fps


class QRcode(threading.Thread):
    def on_error_reveived(self, e):
        self.on_error(e)

    def __init__(self, serdev, callback=None, refresh_callback=CameraPublisher.ALWAYS_REFRESH,
                 freq_callback=0.1, baudrate=115200, timeout=1):
        print "Init QRcode"
        super(QRcode, self).__init__()
        self.on_error = None
        self._serdev = serdev
        self._stop_event = threading.Event()

        try:
            self.publisher = CameraPublisher.CameraPublisher(
                serdev, callback, refresh_callback, freq_callback, baudrate, timeout)
            # print "QRCODE set_on_error " + str(self.on_error_reveived)
            self.publisher.set_on_error(self.on_error_reveived)
        except serial.serialutil.SerialException as e:
            raise serial.SerialException(e)
        print "Init QRcode done"

    def start(self):
        print "Start QRcode"
        video_num = 1
        set_guvcview_fps(30, video_num)
        self._gvcview = subprocess.Popen([
            'guvcview', '-d', '/dev/video%d' % video_num,
            '-g', 'none', '-a', 'none', '-f', 'YUYV',
            '-x', '320x240'], #, '-F', '7.5'],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        self.publisher.start()
        self.publisher.read_cfg('QRcode.cfg')
        print "QRcode started"

    def close(self):
        print "Close QRcode"
        self.publisher.close()
        self._stop_event.set()
        print "QRcode closed"

    def get_value(self):
        return self.publisher.get_value()

    def set_on_error(self, on_error):
        self.on_error = on_error
