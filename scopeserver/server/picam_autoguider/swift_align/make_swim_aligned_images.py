#!/usr/bin/env python3

import cv2
import numpy as np
import glob
import subprocess as sp
import os


# get list of jpeg images to be aligned:
im_dir_input = './src/'
jpg_files = sorted(glob.glob(im_dir_input + '*.jpg'))

# choose base image for alignment
base_im = 0
im0_fn = jpg_files.pop(base_im)
im0 = cv2.imread(im0_fn,cv2.IMREAD_UNCHANGED)
#im0 = (cv2.cvtColor(im0,cv2.COLOR_BGR2GRAY)/256.).astype('uint8')

im_fn_aligned = os.path.join('./aligned', os.path.basename(im0_fn))
cv2.imwrite(im_fn_aligned,im0)

# Find size of image1
sz = im0.shape
print('\naligning to image: ' + im0_fn)
print('image size: ' + str(sz))

swim_cmd_template = 'swim 2048 -i 2 -w -0.61 -k keep.JPG %s %s' 
warp_matrix = np.eye(2, 3, dtype=np.float32)

for im_fn in jpg_files:
  swim_cmd = swim_cmd_template % (im0_fn,im_fn)
  swim_log = sp.run([swim_cmd],shell=True,stdout=sp.PIPE,stderr=sp.PIPE)
  swim_stdout = swim_log.stdout.decode('utf-8')
  swim_stderr = swim_log.stdout.decode('utf-8')
  toks = swim_stdout.split()
  print('swim output: \n  ' + swim_stdout)
  snr = float(toks[0][0:-1])
  dx = float(toks[8][1:])
  dy = float(toks[9])
  warp_matrix[0,2]=dx
  warp_matrix[1,2]=dy
  print('%s : swim match:  SNR: %g  dX: %g  dY: %g' % (im_fn, snr, dx, dy))
  print(warp_matrix)
  im_fn_aligned = os.path.join('./aligned', os.path.basename(im_fn))
  im2 =  cv2.imread(im_fn,cv2.IMREAD_UNCHANGED)
  im2_aligned = cv2.warpAffine(im2, warp_matrix, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP);
  print('writing aligned image: %s' % (im_fn_aligned))
  cv2.imwrite(im_fn_aligned,im2_aligned)

exit(0)

