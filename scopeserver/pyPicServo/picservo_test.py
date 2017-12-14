#!/usr/bin/env python3

import nmccom

net = nmccom.NmcNet()

net.NmcInit(baudrate=230400)

net.ServoSetGain(1, 100, 1000, 0, 0, 255, 0, 4000, 1, 0, 1)

# ServoOn
net.ServoStopMotor(1)
net.ServoStopAbrupt(1)
net.ServoResetPos(1)

# Go
net.ServoLoadTraj(1, nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 2000, 10000, 100, 0)

