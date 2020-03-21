#!/usr/bin/env python3

import picamera
import numpy as np
import cv2
import io
import shutil
import time

print('\nConfiguring Camera...\n')
cam = picamera.PiCamera()
cam.resolution = (3280,2464)
cam.sensor_mode = 2
cam.rotation = 180
cam.vflip = True
exposure = 1.0
cam.framerate = 1/exposure
cam.shutter_speed = int(exposure*1e6/1.0)
cam.iso = 800
cam.meter_mode = 'average'
cam.exposure_mode = 'night'
cam.image_denoise = False
time.sleep(5)
cam.exposure_mode = 'off'

cam.capture('./init_image.jpg', use_video_port=True, quality=75)
print('\nCamera ready:')
print('  Exposure: %s' % (str(cam.exposure_speed)))
print('  ISO: %s' % (str(cam.iso)))
print('  Digital gain: %s' % (str(float(cam.digital_gain))))
print('  Analog gain: %s\n' % (str(float(cam.analog_gain))))

stream = io.BytesIO()
n = 0
num = 500
t0 = time.time()
tp = t0
#for _ in cam.capture_continuous(stream, format='jpeg', use_video_port=False):
for _ in cam.capture_continuous(stream, format='jpeg', use_video_port=True):
  # Truncate the stream to the current position (in case
  # prior iterations output a longer image)
  stream.truncate()
  stream.seek(0)
  s = stream.getvalue()
  d = np.frombuffer(s, dtype=np.uint8)
  img = cv2.imdecode(d, cv2.IMREAD_GRAYSCALE)
  ofn = './images/image_%02d.jpg' % (n)
#  cv2.imwrite(ofn, img)
  stream.seek(0)
  stream.truncate()
  tn = time.time()
  dt = tn - tp
  tp = tn

  n += 1
  print('  n: %d  dt: %g' % (n, dt))
  if n >= num:
    break

cam.close()
