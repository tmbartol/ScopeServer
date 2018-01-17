#!/usr/bin/env python3

import sys
import signal
import nmccom
#import nmccom2 as nmccom
import time


def signal_handler(signal, frame):
  print('\n\nYou pressed Ctrl+C!\n')
  t2 = time.time()
  print('Total Ops:  %d' % (ops))
  print('Total Time:  %d seconds' % (t2-t1))
  print('Ops per second:  %.4g' % (float(ops)/(t2-t1)))
  print('Bytes sent:      %d' % (ra_mod.bytes_sent))
  print('Bytes received:  %d' % (ra_mod.bytes_received))
  print('Communication bytes per second:  %.4g' % (float(ra_mod.bytes_sent + ra_mod.bytes_received)/(t2-t1)))
  print('Send Errors: %d   Receive Errors: %d' % (net.send_errors, net.receive_errors))

  # Shutdown NMC Network
  net.Shutdown()

  sys.exit(0)


net = nmccom.NmcNet()

net.Initialize(['RA'],baudrate=230400)

ra_mod = net.modules['RA']

ra_mod.ServoIOControl()

ra_mod.verbosity = 1


t1 = time.time()
signal.signal(signal.SIGINT, signal_handler)
print('\n\n>>>>> Press Ctrl+C to Quit <<<<<\n\n')

ra_mod.ServoResetPos()
i = 0
ops = 0
while True:
  pos = ra_mod.ServoGetPos()
  i+=1
  ops+=1
  if i%100 == 0:
    ra_mod.PrintFullStatusReport()
    ops+=1
#    print('pos: %d' % (pos))


'''
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

i=0
pos = 0
ops = 0
ra_mod.bytes_sent = 0
ra_mod.bytes_received = 0

t1 = time.time()
signal.signal(signal.SIGINT, signal_handler)
print('\n\n>>>>> Press Ctrl+C to Quit <<<<<\n\n')
# signal.pause()

while True:
  ra_mod.ServoSetPos(pos)
  ra_mod.ReadFullStatus()
  pos += 1
  ops += 2
  if i%1000 == 0:
    ra_mod.PrintFullStatusReport()
    ops += 1
#  ra_mod.NoOp()
#  print(i, ra_mod.response)
#  ops += 1
  i+=1

'''


