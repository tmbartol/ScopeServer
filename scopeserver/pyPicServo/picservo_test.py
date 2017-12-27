#!/usr/bin/env python3

import nmccom

net = nmccom.NmcNet()

net.Initialize(['RA'],baudrate=230400)

ra_mod = net.modules['RA']

ra_mod.ServoSetGain(100, 1000, 0, 0, 255, 0, 4000, 1, 0, 1)

# ServoOn
ra_mod.ServoStopMotor()
ra_mod.ServoStopAbrupt()
ra_mod.ServoResetPos()

# Go
resp, checksum_error = ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 2000, 10000, 100, 0)

#while (not (resp[0] & nmccom.MOVE_DONE)):
#  resp, checksum_error = ra_mod.NoOp()

print('Move Done: ',  (resp[0] & nmccom.MOVE_DONE))


# ReadPos
ra_mod.DefineStatusData(nmccom.SEND_POS)
pos, checksum_error = ra_mod.ServoGetPos()
print('Current position = %d' % (pos))

ra_mod.ServoSetPos(20000)
pos, checksum_error = ra_mod.ServoGetPos()
print('Current position = %d' % (pos))

#ra_mod.DefineStatusData(0x00)
#for i in range(100):
#  resp, checksum_error = ra_mod.NoOp()
#  print(i, resp)

# Shutdown NMC Network
net.Shutdown()

