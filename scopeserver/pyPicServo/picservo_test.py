#!/usr/bin/env python3

import nmccom

net = nmccom.NmcNet()

net.NmcInit(['RA'],baudrate=230400)

net.ServoSetGain(1, 100, 1000, 0, 0, 255, 0, 4000, 1, 0, 1)

# ServoOn
net.ServoStopMotor(1)
net.ServoStopAbrupt(1)
net.ServoResetPos(1)

# Go
resp, checksum_error = net.ServoLoadTraj(1, nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 2000, 10000, 100, 0)

#while (not (resp[0] & nmccom.MOVE_DONE)):
#  resp, checksum_error = net.NmcNoOp(1)

print('Move Done: ',  (resp[0] & nmccom.MOVE_DONE))


# ReadPos
net.NmcDefineStatusData(1, nmccom.SEND_POS)
pos, checksum_error = net.ServoGetPos(1)
print('Current position = %d' % (pos))

net.ServoSetPos(1,20000)
pos, checksum_error = net.ServoGetPos(1)
print('Current position = %d' % (pos))

#net.NmcDefineStatusData(1, 0x00)
#for i in range(100):
#  resp, checksum_error = net.NmcNoOp(1)
#  print(i, resp)

# Shutdown NMC Network
net.NmcShutdown()

