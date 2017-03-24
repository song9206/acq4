# -*- coding: utf-8 -*-
import time
import numpy as np
from PyQt4 import QtGui, QtCore
from ..Stage import Stage, MoveFuture, StageInterface
from acq4.drivers.LuigsNeumann import LuigsNeumann as LuigsNeumannDriver
from acq4.util.Mutex import Mutex
from acq4.util.Thread import Thread
from acq4.pyqtgraph import debug, ptime, SpinBox


class LuigsNeumann(Stage):
    """
    A Luigs & Neumann motorized device.


        port: <serial port>  # eg. 'COM1' or '/dev/ttyACM0'
        name: <string>  # eg. 'SliceScope' or 'MicroStar 2'
        baudrate: <int>  #  may be 9600 or 38400

    The optional 'baudrate' parameter is used to set the baudrate of the device.
    Both valid rates will be attempted when initially connecting.
    """
    def __init__(self, man, config, name):
        port = config.pop('port')
        name = config.pop('name', name)
        self.scale = config.pop('scale', (1e-6, 1e-6, 1e-6))
        self.device_number = config.pop('device_number')
        self.dev = LuigsNeumannDriver.getDriver(port=port,
                                                device=self.device_number)

        self._lastMove = None
        man.sigAbortAll.connect(self.abort)

        Stage.__init__(self, man, config, name)

        # clear cached position for this device and re-read to generate an initial position update
        self._lastPos = None
        self.getPosition(refresh=True)

        self.userSpeed = 30000  # FIXME: placeholder

        # thread for polling position changes
        self.monitor = MonitorThread(self)
        self.monitor.start()

    def capabilities(self):
        """Return a structure describing the capabilities of this device"""
        if 'capabilities' in self.config:
            return self.config['capabilities']
        else:
            return {
                'getPos': (True, True, True),
                'setPos': (True, True, True),
                'limits': (False, False, False),
            }

    def stop(self):
        """Stop the manipulator immediately.
        """
        with self.lock:
            self.dev.stop(device=self.device_number)
            if self._lastMove is not None:
                time.sleep(0.2)  # stopping might take a moment
                self._lastMove._stopped()
            self._lastMove = None

    def abort(self):
        """Stop the manipulator immediately.
        """
        self.dev.stop(device=self.device_number)
        if self._lastMove is not None:
            time.sleep(0.2)  # stopping might take a moment
            self._lastMove._stopped()
            self._lastMove = None

    def _getPosition(self):
        # Called by superclass when user requests position refresh
        with self.lock:
            pos = self.dev.getPos(device=self.device_number)
            pos = [pos[i] * self.scale[i] for i in (0, 1, 2)]
            if pos != self._lastPos:
                self._lastPos = pos
                emit = True
            else:
                emit = False

        if emit:
            # don't emit signal while locked
            self.posChanged(pos)

        return pos

    def targetPosition(self):
        with self.lock:
            if self._lastMove is None or self._lastMove.isDone():
                return self.getPosition()
            else:
                return self._lastMove.targetPos

    def quit(self):
        self.monitor.stop()
        Stage.quit(self)

    def _move(self, abs, rel, speed, linear):
        with self.lock:
            if self._lastMove is not None and not self._lastMove.isDone():
                self.stop()
            pos = self._toAbsolutePosition(abs, rel)
            speed = self._interpretSpeed(speed)

            self._lastMove = LuigsNeumannMoveFuture(self, pos, speed)
            return self._lastMove


class MonitorThread(Thread):
    """Thread to poll for manipulator position changes.
    """
    def __init__(self, dev):
        self.dev = dev
        self.lock = Mutex(recursive=True)
        self.stopped = False
        self.interval = 0.3
        
        Thread.__init__(self)

    def start(self):
        self.stopped = False
        Thread.start(self)

    def stop(self):
        with self.lock:
            self.stopped = True

    def setInterval(self, i):
        with self.lock:
            self.interval = i
    
    def run(self):
        minInterval = 100e-3
        interval = minInterval
        lastPos = None
        while True:
            try:
                with self.lock:
                    if self.stopped:
                        break
                    maxInterval = self.interval

                pos = self.dev._getPosition()  # this causes sigPositionChanged to be emitted
                if pos != lastPos:
                    # if there was a change, then loop more rapidly for a short time.
                    interval = minInterval
                    lastPos = pos
                else:
                    interval = min(maxInterval, interval*2)

                time.sleep(interval)
            except:
                debug.printExc('Error in LuigsNeumann monitor thread:')
                time.sleep(maxInterval)
                

class LuigsNeumannMoveFuture(MoveFuture):
    """Provides access to a move-in-progress on a Luigs & Neumann manipulator.
    """
    def __init__(self, dev, pos, speed):
        MoveFuture.__init__(self, dev, pos, speed)
        self._interrupted = False
        self._errorMSg = None
        self._finished = False
        pos = [pos[i] / dev.scale[i] for i in (0, 1, 2)]
        with self.dev.dev.lock:
            self.dev.dev.moveTo(device=dev.device_number, pos=pos)
            time.sleep(0.25)
        
    def wasInterrupted(self):
        """Return True if the move was interrupted before completing.
        """
        return self._interrupted

    def isDone(self):
        """Return True if the move is complete.
        """
        return self._getStatus() != 0

    def _getStatus(self):
        # check status of move unless we already know it is complete.
        # 0: still moving; 1: finished successfully; -1: finished unsuccessfully
        if self._finished:
            if self._interrupted:
                return -1
            else:
                return 1
        if self.dev.dev.isMoving(device=self.dev.device_number):
            # Still moving
            return 0
        # did we reach target?
        pos = self.dev._getPosition()
        dif = ((np.array(pos) - np.array(self.targetPos))**2).sum()**0.5
        if dif < 2.5e-6:
            # reached target
            self._finished = True
            return 1
        else:
            # missed
            self._finished = True
            self._interrupted = True
            self._errorMsg = "Move did not complete (target=%s, position=%s, dif=%s)." % (self.targetPos, pos, dif)
            return -1

    def _stopped(self):
        # Called when the manipulator is stopped, possibly interrupting this move.
        status = self._getStatus()
        if status == 1:
            # finished; ignore stop
            return
        elif status == -1:
            self._errorMsg = "Move was interrupted before completion."
        elif status == 0:
            # not actually stopped! This should not happen.
            raise RuntimeError("Interrupted move but manipulator is still running!")
        else:
            raise Exception("Unknown status: %s" % status)

    def errorMessage(self):
        return getattr(self, '_errorMsg', 'Unknown error')

