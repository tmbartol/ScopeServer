#!/usr/bin/env python3

import numpy as np
import scipy.stats as sps
import subprocess as sp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import cv2
import astropy
from astropy.stats import sigma_clipped_stats, mad_std
from astropy.visualization import SqrtStretch
from astropy.visualization.mpl_normalize import ImageNormalize
from photutils import  DAOStarFinder, CircularAperture
from photutils.psf.groupstars import DAOGroup
import glob


# Do Linear Regression of X,Y data
def lin_fit(x,y):

  (m,b,r,p,stderr) = sps.linregress(x,y)
  print('linear regression:')
  print('  slope:',m)
  print('  intercept:',b)
  print('  r:',r)
  print('  p:',p)
  print('  stderr:',stderr)
  print('')

  return(m,b,r,p,stderr)


def find_guide_star(img_fn,img_ref_fn):

  w = 128
  h = 128

  img_color = cv2.imread(img_fn,cv2.IMREAD_UNCHANGED)
  data = np.flipud(cv2.cvtColor(img_color,cv2.COLOR_BGR2GRAY))
  print(str(data.shape))

  mean, median, std = sigma_clipped_stats(data, sigma=3.0)
  print((mean, median, std))
  bkg_sigma = mad_std(data)
  print(bkg_sigma)

  daofind = DAOStarFinder(fwhm=9.0, sharphi=0.65, threshold=20.0*bkg_sigma, exclude_border=True)

  sources = daofind(data - median)
  for col in sources.colnames:  
      sources[col].info.format = '%.8g'  # for consistent table output

  sources.sort(['peak'])
  sources.reverse()
  
  print('')
  print('All sources:')
  print(sources)  

  print('')
  print('Guide Source:')
  print(sources[0])  

  guide_star_pos = np.array([sources['xcentroid'][0],sources['ycentroid'][0]])
  guide_star_rect = np.array([guide_star_pos-[w/2.,h/2.],w,h])

  image_crop_rect(data, guide_star_rect, img_ref_fn)

  # Show image with identified sources
  fig,ax = plt.subplots(1)
  norm = ImageNormalize(stretch=SqrtStretch())
  ax.imshow(data, cmap='Greys', origin='lower', norm=norm)

  rect = patches.Rectangle(guide_star_rect[0],guide_star_rect[1],guide_star_rect[2],linewidth=1,edgecolor='r',facecolor='none')
  ax.add_patch(rect)

  positions = np.transpose((sources['xcentroid'], sources['ycentroid']))
  apertures = CircularAperture(positions, r=5.)
  apertures.plot(color='blue', lw=1.5, alpha=0.5)
  plt.show()

  return (guide_star_pos, guide_star_rect)


def image_crop_rect(img,rect,img_fn):
  x = int(rect[0][0])
  y = int(rect[0][1])
  w = int(rect[1])
  h = int(rect[2])
  img_cropped = img[y:(y+h), x:(x+w)]
  cv2.imwrite(img_fn,img_cropped)




# Main Routine:

img_dir = './images_2/'
swim_dir = './swim_tmp/'

img_fns = sorted(glob.glob(img_dir + '*'))

print('\nAnalyzing %d images\n' % (len(img_fns)))

img_ref_fn = swim_dir + 'guide_ref.jpg'
img_update_fn = swim_dir + 'guide_update.jpg'

t0 = float('.'.join(img_fns[0].split('.')[2:4]))

(guide_star_pos, guide_star_rect) = find_guide_star(img_fns[0],img_ref_fn)

swim_cmd_template = 'swim 128 -i 2 -w -0.61 -k keep.JPG %s %s'

guide_star_dat = np.empty((0,5))
guide_star_dat = np.append(guide_star_dat,[0,0,0,0,0]).reshape(-1,5)
dx = 0
dy = 0
w = 128
h = 128
for img_fn in img_fns[1:]:

  t = float('.'.join(img_fn.split('.')[2:4]))-t0

  print('\nAnalyzing %s' % (img_fn))
  img_color = cv2.imread(img_fn,cv2.IMREAD_UNCHANGED)
  data = np.flipud(cv2.cvtColor(img_color,cv2.COLOR_BGR2GRAY))
  print(str(data.shape))

  guide_update_rect = np.array([guide_star_pos+[dx,dy]-[w/2.,h/2.],w,h])
  image_crop_rect(data,guide_update_rect,img_update_fn)

  swim_cmd = swim_cmd_template % (img_ref_fn,img_update_fn)
  swim_log = sp.run([swim_cmd],shell=True,stdout=sp.PIPE,stderr=sp.PIPE)
  swim_stdout = swim_log.stdout.decode('utf-8')
  swim_stderr = swim_log.stdout.decode('utf-8')
  toks = swim_stdout.split()
  print('swim output: \n  ' + swim_stdout)
  snr = float(toks[0][0:-1])
  dxi = float(toks[8][1:])
  dyi = float(toks[9])
  dx += dxi
  dy += dyi
  print('%s : swim match:  SNR: %g  dX: %g  dY: %g' % (img_fn, snr, dxi, dyi))

  guide_star_dat = np.append(guide_star_dat,[t,dx,dy,dxi,dyi]).reshape(-1,5)

  print('')


# Make plot of RA guide error
t = guide_star_dat[:,0]
x = guide_star_dat[:,1]
y = guide_star_dat[:,2]
xi = guide_star_dat[:,3]
yi = guide_star_dat[:,4]
v = np.array((x,y)).transpose()

(m,b,r,p,stderr) = lin_fit(x,y)
theta = np.arctan(-m)
rot_mat = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta),np.cos(theta)]])
vt = np.matmul(rot_mat,v.transpose()).transpose()
xt = vt[:,0]
yt = vt[:,1]

(m,b,r,p,stderr) = lin_fit(xt,yt)
theta = np.arctan(-m)
rot_mat = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta),np.cos(theta)]])
vt = np.matmul(rot_mat,vt.transpose()).transpose()
xt = vt[:,0]
yt = vt[:,1]
(m,b,r,p,stderr) = lin_fit(xt,yt)
xt = xt-(xt[0])
yt = yt-b

fov_pix = (3600/3280)*2*np.arctan(3.68/(2*240))*180/np.pi

fig,ax = plt.subplots(1)
ax.scatter(t,yt*fov_pix)
ax.plot([0,480],[-0.5,-0.5],dashes=[4,2],color='g')
ax.plot([0,480],[0.5,0.5],dashes=[4,2],color='g')
ax.plot([0,480],[-2.5,-2.5],dashes=[4,2],color='b')
ax.plot([0,480],[2.5,2.5],dashes=[4,2],color='b')
ax.plot([0,480],[-3.5,-3.5],dashes=[4,2],color='r')
ax.plot([0,480],[3.5,3.5],dashes=[4,2],color='r')
ax.axis([0,480,-10,10])
ax.set_xlabel(r't (s) ',labelpad=1)
ax.set_ylabel(r'RA tracking error (arcsec) ',labelpad=1)
plt.show()

fig,ax = plt.subplots(1)
ax.scatter(t,xi*fov_pix,color='r')
ax.scatter(t,yi*fov_pix,color='b')
ax.plot([0,480],[-0.5,-0.5],dashes=[4,2],color='g')
ax.plot([0,480],[0.5,0.5],dashes=[4,2],color='g')
ax.axis([0,480,-10,10])
ax.set_xlabel(r't (s) ',labelpad=1)
ax.set_ylabel(r'guide corrections (arcsec) ',labelpad=1)
plt.show()

