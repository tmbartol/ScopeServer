#!/usr/bin/env python3.5

import rawpy
import rawpy.enhance
import cv2
import imageio
import glob
import os

raw_files = sorted(glob.glob('./raw_input/*.NEF'))

print('Finding bad pixels...')
bad_pixels = rawpy.enhance.find_bad_pixels(raw_files)
rawpy.enhance.save_dcraw_bad_pixels('bad_pixels.dcraw', bad_pixels)
print('Done finding bad pixels.')

for raw_file in raw_files:
  print('Processing raw image: %s' % (raw_file))
  raw_image = rawpy.imread(raw_file)
  rawpy.enhance.repair_bad_pixels(raw_image, bad_pixels, method='median')
#  rgb_linear = raw_image.postprocess(gamma=(1,1), no_auto_bright=True, output_bps=8)
  rgb_linear = raw_image.postprocess(no_auto_bright=True, output_bps=8)
  jpg_file = './linear_jpgs/' + os.path.splitext(os.path.split(raw_file)[1])[0] + '.jpg'
#  imageio.imsave(tiff_file, rgb_linear)
  bgr_linear = cv2.cvtColor(rgb_linear,cv2.COLOR_RGB2BGR)
  cv2.imwrite(jpg_file, bgr_linear)
