import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from acq4.drivers.LuigsNeumann import LuigsNeumann

ps = LuigsNeumann(port='COM3')
ps.addDevice(1)
ps.addDevice(2)
ps.addDevice(3)

for dev in range(1, 4):
    print 'Device %d:' % dev
    print '\tMotor: %s' % str([ps.motor[(dev, a)] for a in range(1, 4)])
    print '\tPitch: %s' % str([ps.pitch[(dev, a)] for a in range(1, 4)])
    print '\tSpeed (Fast) in mm/s: %s' % str(['%.3f' % (ps.getSpeed(dev, a, fast=True)*1000) for a in range(1, 4)])
    print '\tSpeed (Slow) in mm/s: %s' % str(['%.3f' % (ps.getSpeed(dev, a, fast=False)*1000) for a in range(1, 4)])
    print '\tPosition: %s' % str(ps.getPos(device=dev))

# Make sure this is safe before uncommenting!
# ps.moveTo(device=1, pos=[0, None, None], fast=True)
# time.sleep(0.25)  # wait a moment so that the movement starts
# while ps.isMoving(device=1):
#     print ps.getPos(device=1)
#     time.sleep(0.5)
#
# print 'final position', ps.getPos(device=1)
