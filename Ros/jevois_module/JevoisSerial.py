import serial 
import time

class SerWrapper:
    def __init__(self, ser):
        self.ser = ser
    def write(self, *args, **kw):
        print 'SerWrapper::Write:', args, kw
        out = self.ser.write(*args, **kw)
        return out
    
    def read(self, *args, **kw):
        # print 'SerWrapper::Read:', args, kw
        out = self.ser.read(*args, **kw)
        # print 'SerWrapper::Out', out
        return out
    def __getattr__(self, attr):
        return getattr(self.ser, attr)
    def __repr__(self):
        return 'SerWrapper(%s)' % self.ser
class JevoisSerial:
    """Class sending and reading information on the serial port"""

    def __init__(self, serdev, baudrate=115200, timeout=1):
        print "Init JevoisSerial"
        self.serdev = serdev
        self._serdev = serdev
        self._baudrate = baudrate
        self._timeout = timeout
        self.ser = None
        print "Init JevoisSerial done"

    def start(self):
        print "start JEvoisSerial..."
        count = 0
        while count < 5:
            try:
                print "Count = " + str(count)
                # self.ser = SerWrapper(serial.Serial(self._serdev, self._baudrate,
                #                                    timeout=self._timeout))
                self.ser = serial.Serial(self._serdev, self._baudrate,
                                         timeout=self._timeout)
                print "JevoisSerial started"
                return self.ser
            except serial.serialutil.SerialException, e:
                if count == 4:
                    print str(e)
                    self.close()
                    raise serial.SerialException
                count += 1
                time.sleep(1)

    def send_command(self, cmd):
        self.ser.write(cmd + '\n')
        print "Command sent:" + cmd

    def get_ser(self):
        return self.ser

    def read_cfg(self, cfgpath):
        print "Reading cfg file..."
        cfg = open(cfgpath, "r")
        for i, line in enumerate(cfg):
            if line[0] != "#":
                try:
                    self.send_command(line)
                except serial.serialutil.SerialException:
                    print 'error line' + str(i)
                    pass

    def close(self):
        print "JevoisSerial close"
        if self.ser is not None:
            self.ser.close()
