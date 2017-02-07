"""
Driver for communicating with LuigsNeumann motorized devices by serial interface.
"""
from __future__ import division
import binascii
import ctypes
import serial, struct, time, collections
import numpy as np

from acq4.util.Mutex import RecursiveMutex as RLock
from ..SerialDevice import SerialDevice, TimeoutError, DataError


def crc_16(butter, length):
    # Calculate CRC-16 checksum based on the data sent
    #
    crc_polynom = 0x1021
    crc = 0
    n = 0
    lll = length
    while (lll > 0):
        crc = crc ^ butter[n] << 8
        for _ in range(8):
            if (crc & 0x8000):
                crc = crc << 1 ^ crc_polynom
            else:
                crc = crc << 1
        lll -= 1
        n += 1
    crc_high = ctypes.c_ubyte(crc >> 8)
    crc_low = ctypes.c_ubyte(crc)
    return (crc_high.value, crc_low.value)


class LuigsNeumann(SerialDevice):
    """
    Provides interface to a LuigsNeumann manipulator.

    This can be initialized either with the com port name or with the string description
    of the device.

    Examples::

        dev = LuigsNeumann('com4')
        print dev.getPos()
        dev.moveTo([10e-3, 0, 0], 'fast')

        # search for device with this description
        dev2 = LuigsNeumann(name='SliceScope')
    """
    openDevices = {}
    MOTOR = {0: 'P310',
             1: 'P430',
             2: 'P530',
             3: 'PK223',
             4: 'PK233',
             5: 'ST2018',
             6: 'PK244P',
             7: 'PK244M'}
    MOTOR_STEPS = {'P310': 60,
                   'P430': 100,
                   'P530': 100,
                   'PK223': 200,
                   'PK233': 200,
                   'ST2018': 200,
                   'PK244P': 200,
                   'PK244M': 400}
    PITCH = {0: 0.020*1e-3,
             1: 0.050*1e-3,
             2: 0.100*1e-3,
             3: 0.125*1e-3,
             4: 0.175*1e-3,
             5: 0.350*1e-3,
             6: 0.400*1e-3,
             7: 0.500*1e-3,
             8: 1.000*1e-3,
             9: 2.000*1e-3,
             10: 2.97*1e-3  #TODO: Says 0.297*1e-3 in the docs...
             }
    SLOW_VELOCITY = {0: 0.000017,
                     1: 0.000040,
                     2: 0.000141,
                     3: 0.000260,
                     4: 0.001280,
                     5: 0.02630,
                     6: 0.05070,
                     7: 0.010200,
                     8: 0.025100,
                     9: 0.060100,
                     10: 0.173000,
                     11: 0.332000,
                     12: 0.498000,
                     13: 0.664000, #TODO: Says 0.066400 in the docs...
                     14: 0.996000,
                     15: 1.328000}
    FAST_VELOCITY = {0: 0.66,
                     1: 1.73,
                     2: 2.63,
                     3: 3.79,
                     4: 4.67,
                     5: 5.68,
                     6: 6.33,
                     7: 7.81,
                     8: 8.47,
                     9: 9.52,
                     10: 10.42,
                     11: 11.36,
                     12: 12.32,
                     13: 13.23,
                     14: 14.29,
                     15: 15.15}

    def __init__(self, port, baudrate=115200):
        self.lock = RLock()

        self.port = self.normalizePortName(port)
        self.n_devices = 0
        if self.port in self.openDevices:
            raise RuntimeError("Port %s is already in use by %s" % (port, self.openDevices[self.port]))

        SerialDevice.__init__(self, port=self.port, baudrate=baudrate)
        # Make sure the read buffer does not contain anything anymore from a
        # previous run
        try:
            self.read(1000, timeout=1.0)
        except TimeoutError:
            pass
        self.motor = {}
        self.pitch = {}

        # Make sure to call addDevice for each device!

    n_axes = property(lambda self: self.n_devices * 3)

    def addDevice(self, device):
        self.n_devices = max([self.n_devices, device])
        # Make sure that all axis are switched on (and exist...)
        for axis in range((device-1)*3 + 1, device*3 + 1):
            if not self.getPower(axis):
                self.setPower(axis)
        for axis in range(1, 4):
            self.motor[(device, axis)] = self.getMotor(device, axis)
            self.pitch[(device, axis)] = self.getPitch(device, axis)

    @classmethod
    def getDriver(cls, port, device):
        port = cls.normalizePortName(port)
        if port in LuigsNeumann.openDevices:
            driver = LuigsNeumann.openDevices[port]
        else:
            driver = LuigsNeumann(port)
            LuigsNeumann.openDevices[port] = driver

        driver.addDevice(device)

        return driver

    def close(self):
        port = self.port
        SerialDevice.close(self)
        del LuigsNeumann.openDevices[port]

    def send(self, ID, data, nbytes_answer, timeout=1.0):
        '''
        Send a command to the controller
        '''
        high, low = crc_16(data, len(data))

        # Create hex-string to be sent
        # <syn><ID><byte number>
        send = '16' + ID + '%0.2X' % len(data)

        # <data>
        # Loop over length of data to be sent
        for i in range(len(data)):
            send += '%0.2X' % data[i]

        # <CRC>
        send += '%0.2X%0.2X' % (high, low)

        # Convert hex string to bytes
        sendbytes = binascii.unhexlify(send)
        with self.lock:
            self.write(sendbytes)

            if nbytes_answer > 0:
                # Expected response: <ACK><ID><byte number><data><CRC>
                # We just check the first two bytes
                expected = binascii.unhexlify('06' + ID)

                answer = self.read(nbytes_answer+6, timeout=timeout)

                if answer[:len(expected)] != expected :
                    msg = "Expected answer '%s', got '%s' " \
                          "instead" % (binascii.hexlify(expected),
                                       binascii.hexlify(answer[:len(expected)]))
                    raise serial.SerialException(msg)
                # We should also check the CRC + the number of bytes
                # Do several reads; 3 bytes, n bytes, CRC
                return answer[4:4+nbytes_answer]
            else:
                return None

    def checkAxis(self, axis):
        assert axis <= self.n_axes, 'Only %d axes supported (illegal axis: %d)' % (self.n_axes, axis)

    def groupAddress(self):
        assert self.n_devices <= 6
        # This selects all axes, for a subset it would be more complex to calculate
        all_axes = 2**self.n_axes - 1
        # The group address is fixed at 9 bites
        address = binascii.unhexlify('%.18x' % all_axes)
        assert len(address) == 9

    def setPower(self, axis, on=True):
        """Switch a given axis an or off"""
        self.checkAxis(axis)
        ID = '0035' if on else '0034'
        try:
            ret = self.send(ID, [axis], 1)
        except TimeoutError as ex:
            on_off = 'on' if on else 'off'
            raise RuntimeError('Could not switch axis %d %s, no response.' % (axis, on_off))

    def getPower(self, axis):
        self.checkAxis(axis)
        try:
            ret = struct.unpack('b', self.send('011E', [axis], 1))[0]
            return ret == 1
        except TimeoutError as ex:
            raise RuntimeError('Could not get status for axis %d, no response.' % axis)

    def getMotor(self, device, axis):
        """Get a string description of the motor controlling the given axis.
        """
        axis = (device - 1) * 3 + axis
        ret = struct.unpack('b', self.send('014B', [axis], 1))[0]
        return LuigsNeumann.MOTOR[ret]

    def getPitch(self, device, axis):
        """Get the pitch (in m) of the given axis"""
        axis = (device - 1) * 3 + axis
        ret = struct.unpack('b', self.send('014D', [axis], 1))[0]
        return LuigsNeumann.PITCH[ret]

    def getSpeed(self, device, axis, fast=True):
        steps = LuigsNeumann.MOTOR_STEPS[self.motor[(device, axis)]]
        if steps != 200:
            raise NotImplementedError('Only motors with 200 steps supported.')
        if fast:
            ID = '012F'
        else:
            ID = '0130'
        ret = struct.unpack('b', self.send(ID, [axis], 1))[0]
        if fast:
            return LuigsNeumann.FAST_VELOCITY[ret] * self.pitch[(device, axis)]
        else:
            return LuigsNeumann.SLOW_VELOCITY[ret] * self.pitch[(device, axis)]


    def getSinglePos(self, device, axis):
        """Get current manipulator position reported by controller in micrometers.
        """
        axis = (device - 1) * 3 + axis
        return struct.unpack('f', self.send('0101', [axis], 4))

    def getPos(self, device):
        axes = list(range((device-1)*3 + 1, device*3 + 1))
        ret = struct.unpack('4b4f', self.send('A101', [0xA0] + axes + [0], 20))
        assert all(r == a for r, a in zip(ret[:3], axes))
        return ret[4:7]

    def moveTo(self, device, pos, fast=True):
        """Set the position of the manipulator.
        
        *pos* must be a list of 3 items, each is either a float representing the desired position
        of the manipulator (in micrometers), or None which indicates the axis should not be moved.

        """
        ID = 'A048' if fast else 'A049'
        currentPos = self.getPos(device=device)
        # fill in Nones with current position
        pos = list(pos)
        for i in range(3):
            if pos[i] is None:
                pos[i] = currentPos[i]

        # Send move command
        axes = list(range((device - 1) * 3 + 1, device * 3 + 1))
        pos += [0.0]  # the command accepts 4 axes, we only set 3
        pos = [b for p in pos for b in bytearray(struct.pack('f', p))]
        self.send(ID, [0xA0] + axes + [0] + pos, 0)

    def moveRelative(self, device, distance, fast=True):
        """Move the manipulator

        *distance* must be a list of 3 items, each a float stating the desired move along this axis
        (i.e. 0.0 for not moving) of the manipulator (in micrometers).
        """
        ID = 'A04A' if fast else 'A04B'
        # Send move command
        axes = list(range((device - 1) * 3 + 1, device * 3 + 1))
        distance += [0.0]  # the command accepts 4 axes, we only set 3
        distance = [b for d in distance for b in bytearray(struct.pack('f', d))]
        self.send(ID, [0xA0] + axes + [0] + distance, 0)

    def zeroPosition(self):
        """Reset the stage coordinates to (0, 0, 0) without moving the stage.
        """
        ID = 'A0F0'
        address = self.groupAddress()
        self.send(ID, address, 0)

    def stop(self, device):
        """Stop moving the manipulator.
        """
        ID = '00FF'
        axes = list(range((device - 1) * 3 + 1, device * 3 + 1))
        for axis in axes:
            self.send(ID, [axis], 0)

    def isMoving(self, device):
        """Return True if the manipulator is moving.
        """
        axes = list(range((device - 1) * 3 + 1, device * 3 + 1))
        data = [0xA0] + axes + [0]
        ret = struct.unpack('20b', self.send('A120', data, 20))
        moving = [ret[6], ret[10], ret[14]]
        return any(moving)
