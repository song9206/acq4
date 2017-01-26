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
    Will attempt to connect at both 9600 and 38400 baud rates.

    Examples::

        dev = LuigsNeumann('com4')
        print dev.getPos()
        dev.moveTo([10e-3, 0, 0], 'fast')

        # search for device with this description
        dev2 = LuigsNeumann(name='SliceScope')
    """
    openDevices = {}

    def __init__(self, port, n_devices, baudrate=115200):
        self.lock = RLock()

        self.port = self.normalizePortName(port)
        self.n_devices = n_devices
        self.n_axes = n_devices * 3
        if self.port in self.openDevices:
            raise RuntimeError("Port %s is already in use by %s" % (port, self.openDevices[self.port]))

        SerialDevice.__init__(self, port=self.port, baudrate=baudrate)

        LuigsNeumann.openDevices[self.port] = self

        # Make sure that all axis are switched on (and exist...)
        for axis in range(1, self.n_axes + 1):
            if not self.getPower(axis):
                self.setPower(axis)

    def close(self):
        port = self.port
        SerialDevice.close(self)
        del LuigsNeumann.openDevices[port]

    def send(self, ID, data, nbytes_answer, timeout=1.0):
        '''
        Send a command to the controller
        '''
        high, low = crc_16(data,len(data))

        # Create hex-string to be sent
        # <syn><ID><byte number>
        send = '16' + ID + '%0.2X' % len(data)

        # <data>
        # Loop over length of data to be sent
        for i in range(len(data)):
            send += '%0.2X' % data[i]

        # <CRC>
        send += '%0.2X%0.2X' % (high,low)

        # Convert hex string to bytes
        sendbytes = binascii.unhexlify(send)

        self.write(sendbytes)

        if nbytes_answer > 0:
            # Expected response: <ACK><ID><byte number><data><CRC>
            # We just check the first two bytes
            expected = binascii.unhexlify('06' + ID)

            answer = self.read(nbytes_answer+6, timeout=timeout)

            if answer[:len(expected)] != expected :
                raise serial.SerialException # TODO: something a bit more explicit!
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

    def getSinglePos(self, device, axis):
        """Get current manipulator position reported by controller in micrometers.
        """
        axis = (device - 1)*3 + axis
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
        with self.lock:
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
            self.send(ID, [0xA0] + axes + [0] + pos , 0)

    def moveRelative(self, device, distance, fast=True):
        """Move the manipulator

        *distance* must be a list of 3 items, each a float stating the desired move along this axis
        (i.e. 0.0 for not moving) of the manipulator (in micrometers).
        """
        ID = 'A04A' if fast else 'A04B'
        with self.lock:
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
        with self.lock:
            ID = '00FF'
            axes = list(range((device - 1) * 3 + 1, device * 3 + 1))
            for axis in axes:
                self.send(ID, [axis], 1)

    def isMoving(self, device):
        """Return True if the manipulator is moving.
        """
        with self.lock:
            axes = list(range((device - 1) * 3 + 1, device * 3 + 1))
            data = [0xA0] + axes + [0]
            ret = struct.unpack('20b', self.send('A120', data, 20))
            moving = [ret[6], ret[10], ret[14]]
            return any(moving)
