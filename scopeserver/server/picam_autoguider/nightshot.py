#!/usr/bin/env python

import picamera
from fractions import Fraction
import time

cam = picamera.PiCamera()
cam.sensor_mode = 3
cam.resolution = (3280,2464)
cam.rotation = 180
#cam.zoom = (0.25,0.25,0.75,0.75)

cam.framerate = Fraction(1,1)
cam.shutter_speed = int(1*1e6/1.0)
cam.iso = 800
cam.meter_mode = 'backlit'
cam.exposure_mode = 'night'
cam.image_denoise = False
time.sleep(3)
cam.exposure_mode = 'off'

cam.capture('./images/foo.jpg',format='jpeg',quality=25)
print('Exposure: ', str(cam.exposure_speed))
print('ISO: ', str(cam.iso))
print('Digital gain: ', str(float(cam.digital_gain)))
print('Analog gain: ', str(float(cam.analog_gain)))

print('')
print('Capturing Sequence...')

cam.capture_sequence(['./images/image%02d.jpg' % i for i in range(5)],quality=25)

print('Exposure: ', str(cam.exposure_speed))
print('ISO: ', str(cam.iso))
print('Digital gain: ', str(float(cam.digital_gain)))
print('Analog gain: ', str(float(cam.analog_gain)))

cam.close()

