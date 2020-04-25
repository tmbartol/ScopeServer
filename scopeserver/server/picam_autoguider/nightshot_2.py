#!/usr/bin/env python3

import picamera
import io
import shutil
import time

cam = picamera.PiCamera()
cam.resolution = (3280,2464)
cam.sensor_mode = 2
cam.rotation = 180
cam.vflip = True

w = 1024.
h = 1024.

wf = w/3280.
hf = h/2464.

#cam.zoom = (0.5-wf/2, 0.5-hf/2, 0.5+wf/2, 0.5+hf/2)

exposure = 1.0

cam.framerate = 1/exposure
cam.shutter_speed = int(exposure*1e6/1.0)
cam.iso = 800
cam.meter_mode = 'average'
cam.exposure_mode = 'night'
cam.image_denoise = False
time.sleep(5)
cam.exposure_mode = 'off'

#cam.capture('foo.jpg',format='jpeg',quality=25)

print('')
print('Capturing sequence...')

n = 0
num = 1
t0 = time.time()
tp = t0
stream = io.BytesIO()
for tmp in cam.capture_continuous(stream, use_video_port=True, format='jpeg'):
  # Truncate the stream to the current position (in case
  # prior iterations output a longer image)
  stream.truncate()
  stream.seek(0)
  f = open('./images/image_%02d.jpg' % (n),'wb')
  shutil.copyfileobj(stream, f, length=131072)
  tn = time.time()
  t = tn - t0
  dt = t - tp
  tp = t
  n += 1
  if n >= num:
    break
  print('  n: %d  dt: %g' % (n, dt))
#  f.close()

print('Total t: %g' % (t))


print('Exposure: ', str(cam.exposure_speed))
print('ISO: ', str(cam.iso))
print('Digital gain: ', str(float(cam.digital_gain)))
print('Analog gain: ', str(float(cam.analog_gain)))

cam.close()
