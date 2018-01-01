#!/usr/bin/env python3

import nmccom
import time

net = nmccom.NmcNet()

net.Initialize(['RA'],baudrate=230400)

ra_mod = net.modules['RA']

ra_mod.verbosity = 1

ra_mod.ServoSetGain(100, 1000, 0, 0, 255, 0, 4000, 1, 0, 1)

# ServoOn
ra_mod.ServoStopMotor()
ra_mod.ServoStopAbrupt()
ra_mod.ServoResetPos()

# Go
ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 2000, 10000, 100, 0)

#while (not (resp[0] & nmccom.MOVE_DONE)):
#  resp, checksum_error = ra_mod.NoOp()

print('Move Done: ',  (ra_mod.response[0] & nmccom.MOVE_DONE))


# ReadPos
ra_mod.DefineStatusData(nmccom.SEND_POS)
pos = ra_mod.ServoGetPos()
print('Current position = %d' % (pos))

ra_mod.ServoSetPos(20000)
pos = ra_mod.ServoGetPos()
print('Current position = %d' % (pos))

ra_mod.DefineStatusData(0x00)
t1 = time.time()
for i in range(100):
  ra_mod.NoOp()
  print(i, ra_mod.response)
t2 = time.time()
print('NoOps per second:  %.4g' % (100.0/(t2-t1)))

print('Send Errors: %d   Receive Errors: %d' % (net.send_errors, net.receive_errors))

# Shutdown NMC Network
net.Shutdown()

