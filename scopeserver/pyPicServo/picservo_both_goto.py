#!/usr/bin/env python3

import sys
import signal
import nmccom
#import nmccom2 as nmccom
import time


def signal_handler(signal, frame):
  print('\n\nYou pressed Ctrl+C!\n')
  t2 = time.time()
  ra_mod.ServoStopMotorOff()
  dec_mod.ServoStopMotorOff()
#  pos = ra_mod.ServoGetPos()
#  cps = pos/(t2-t1)
#  print('Counts per second = %.6g' % (cps))
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

net.Initialize(['RA','Dec'],baudrate=230400)

ra_mod = net.modules['RA']
dec_mod = net.modules['Dec']

ra_mod.verbosity = 1
dec_mod.verbosity = 1


dec_counts = 40000*450
ra_counts = 40000*576
sidereal_day = 86400.0/1.002737909350795
sidereal_rate = ra_counts/sidereal_day
servo_sidereal_rate = int(sidereal_rate*0.000512*2**16)

dec_servo_fast_rate = int(2*50000*0.000512*2**16)
ra_servo_fast_rate = int(2*64000*0.000512*2**16)

ra_mod.ServoIOControl(output_mode=nmccom.PH3_MODE)
dec_mod.ServoIOControl(output_mode=nmccom.PH3_MODE)

t1 = time.time()
signal.signal(signal.SIGINT, signal_handler)
print('\n\n>>>>> Press Ctrl+C to Quit <<<<<\n\n')

#ra_mod.ServoSetGain(200, 800, 200, 100, 255, 0, 4000, 1, 0, 1)
#ra_mod.ServoSetGain(400, 800, 100, 100, 255, 0, 4000, 1, 0, 1)
#ra_mod.ServoSetGain(100, 500, 400, 100, 255, 0, 4000, 1, 0, 1)
ra_mod.ServoSetGain(200, 800, 200, 100, 255, 0, 4000, 1, 0, 1)
dec_mod.ServoSetGain(200, 800, 200, 100, 255, 0, 4000, 1, 0, 1)

# RA ServoOn
ra_mod.ServoStopMotor()
#ra_mod.ServoStopAbrupt()

# Dec ServoOn
dec_mod.ServoStopMotor()

# Go
ra_mod.ServoResetPos()
dec_mod.ServoResetPos()

# Step 1: Run in PWM mode to setup commutation:
#ra_mod.ServoLoadTraj(nmccom.LOAD_PWM | nmccom.START_NOW, pwm=64)
#ra_mod.ServoLoadTraj(nmccom.LOAD_PWM | nmccom.START_NOW | nmccom.REVERSE, pwm=64)


# Step 2: Adjust Kd: 
#ra_mod.ServoSetGain(0, 1200, 0, 0, 255, 0, 4000, 1, 0, 1)
#ra_mod.ServoSetGain(0, 1500, 0, 0, 255, 0, 4000, 1, 0, 1)
#ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.ENABLE_SERVO | nmccom.START_NOW, pos=0)
 
# Shutdown NMC Network
#time.sleep(30)
#net.Shutdown()
#sys.exit(0)


# Step 3: Adjust Kp for best step response:
#ra_mod.ServoSetGain(100, 1200, 0, 0, 255, 0, 4000, 1, 0, 1)
#ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 3000, 83886080, 2147483647, 0)

ops = 0

# Slew Dec at forward fast rate
#dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 30*40000, ra_servo_fast_rate, 400, 0)
dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 1*360*50000, dec_servo_fast_rate, 400, 0)

# Slew RA at forward fast rate
#ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 30*40000, ra_servo_fast_rate, 400, 0)
ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 1*360*64000, ra_servo_fast_rate, 400, 0)

t1 = time.time()
i = 0
ra_mod.NoOp()
dec_mod.NoOp()
while ((not (ra_mod.response[0] & 0x01)) or (not (dec_mod.response[0] & 0x01))):
  if i%200 == 0:
    ra_mod.PrintFullStatusReport()
    dec_mod.PrintFullStatusReport()
    ops+=1
  ra_mod.NoOp()
  dec_mod.NoOp()
  ops+=1
  i+=1

t2 = time.time()

pos = ra_mod.ServoGetPos()
cps = pos/(t2-t1)
print('\npos = %d' % (pos))
print('Counts per second = %.6g\n' % (cps))


'''
# Slew at backward fast rate
ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 0*50000, dec_servo_fast_rate, 400, 0)

t1 = time.time()
i = 0
ra_mod.NoOp()
while not ra_mod.response[0] & 0x01:
  if i%200 == 0:
    ra_mod.PrintFullStatusReport()
    ops+=1
  ra_mod.NoOp()
  ops+=1
  i+=1

t2 = time.time()

pos = ra_mod.ServoGetPos()
cps = pos/(t2-t1)
print('\npos = %d' % (pos))
print('Counts per second = %.6g\n' % (cps))
'''

# Slew RA at sidereal rate
pos = ra_mod.ServoGetPos()
ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, pos + 100*40000, servo_sidereal_rate, 100, 0)


# Slew Dec at sidereal rate
pos = dec_mod.ServoGetPos()
dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, pos + 100*40000, servo_sidereal_rate, 100, 0)

t1 = time.time()
i = 0
ra_mod.NoOp()
dec_mod.NoOp()
while ((not (ra_mod.response[0] & 0x01)) or (not (dec_mod.response[0] & 0x01))):
  if i%200 == 0:
    ra_mod.PrintFullStatusReport()
    dec_mod.PrintFullStatusReport()
    ops+=1
  ra_mod.NoOp()
  dec_mod.NoOp()
  ops+=1
  i+=1

t2 = time.time()

pos = ra_mod.ServoGetPos()
cps = pos/(t2-t1)
print('\npos = %d' % (pos))
print('Counts per second = %.6g\n' % (cps))



ra_mod.ServoStopMotorOff()
dec_mod.ServoStopMotorOff()

sys.exit(0)



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


