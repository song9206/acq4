import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from acq4.drivers.LuigsNeumann import LuigsNeumann

ps = LuigsNeumann(port='COM3')
ps.add_device(1)
ps.add_device(2)
ps.add_device(3)

for dev in range(1, 4):
    print 'Device %d:'
    print '\tMotor: %s' % str([ps.motor[(dev, a)] for a in range(1, 4)])
    print '\tPitch: %s' % str([ps.pitch[(dev, a)] for a in range(1, 4)])
    print '\tSpeed: %s' % str([ps.getSpeed(dev, a) for a in range(1, 4)])
    print '\tPosition: %s' % (dev, ps.getPos(device=dev))

# Make sure this is safe before uncommenting!
# ps.moveTo(device=1, pos=[1400., 4500, None], fast=False)
# time.sleep(0.25)  # wait a moment so that the movement starts
# while ps.isMoving(device=1):
#     print ps.getPos(device=1)
#     time.sleep(0.5)
#
# print 'final position', ps.getPos(device=1)
