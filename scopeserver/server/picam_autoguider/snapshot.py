#!/usr/bin/env python

import picamera
import time

cam = picamera.PiCamera()
cam.resolution = (3280,2464)
cam.rotation = 180
#cam.zoom = (0.25,0.25,0.75,0.75)
cam.iso = 50
cam.meter_mode = 'backlit'
cam.image_denoise = False
time.sleep(1)
#cam.framerate = 1/6.
cam.exposure_mode = 'off'
cam.shutter_speed = int(1e6/1000)
time.sleep(0.25)
print('Exposure: ', str(cam.exposure_speed))
print('ISO: ', str(cam.iso))
print('Digital gain: ', str(float(cam.digital_gain)))
print('Analog gain: ', str(float(cam.analog_gain)))
cam.capture('foo.jpg',format='jpeg',quality=25)

cam.close()
