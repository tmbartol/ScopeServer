#!/usr/bin/env python3.5

import cv2
import numpy as np
import imageio
import glob
import os

png_files = sorted(glob.glob('./aligned_pngs/*.png'))

# Read the 16bit color images to be summed
im0_fn = png_files.pop(0)
print('Reading image: %s' % (im0_fn))
im0 =  cv2.imread(im0_fn,cv2.IMREAD_UNCHANGED).astype('uint32')
print('  Max value: %d' % (im0.max()))

for im_fn in png_files:
  print('Reading image: %s' % (im_fn))
  im =  cv2.imread(im_fn,cv2.IMREAD_UNCHANGED).astype('uint32')
  print('  Max value: %d' % (im.max()))
  im0 = im0 + im

max_val = im0.max()
print('\nSummed image max value: %d' % (max_val))

#clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(32,32))
#img_eq = clahe.apply(im0)
#im0 = img_eq.astype('uint16')

# Normalize Image
scale_val = 1*65535/float(max_val)
im0 = (scale_val*im0).astype('uint16')

max_val = im0.max()
print('Normalized image max value: %d\n' % (max_val))

cv2.imwrite('summed_image.png',im0)

