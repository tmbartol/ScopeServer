#!/usr/bin/env python2.7

"""
======================
Histogram Equalization
======================

This examples enhances an image with low contrast, using a method called
*histogram equalization*, which "spreads out the most frequent intensity
values" in an image [1]_. The equalized image has a roughly linear cumulative
distribution function.

While histogram equalization has the advantage that it requires no parameters,
it sometimes yields unnatural looking images.  An alternative method is
*contrast stretching*, where the image is rescaled to include all intensities
that fall within the 2nd and 98th percentiles [2]_.

.. [1] http://en.wikipedia.org/wiki/Histogram_equalization
.. [2] http://homepages.inf.ed.ac.uk/rbf/HIPR2/stretch.htm

"""

import sys
import os
import glob
import numpy as np

import cv2


#    Plot an image along with its histogram and cumulative histogram.
def plot_img_and_hist(img, axes, bins=256):

    img = img_as_float(img)
    ax_img, ax_hist = axes
    ax_cdf = ax_hist.twinx()

    # Display image
    ax_img.imshow(img, cmap=plt.cm.gray)
    ax_img.set_axis_off()
    ax_img.set_adjustable('box-forced')

    # Display histogram
    ax_hist.hist(img.ravel(), bins=bins, histtype='step', color='black')
    ax_hist.ticklabel_format(axis='y', style='scientific', scilimits=(0, 0))
    ax_hist.set_xlabel('Pixel intensity')
    ax_hist.set_xlim(0, 1)
    ax_hist.set_yticks([])

    # Display cumulative distribution
    img_cdf, bins = exposure.cumulative_distribution(img, bins)
    ax_cdf.plot(bins, img_cdf, 'r')
    ax_cdf.set_yticks([])

    return ax_img, ax_hist, ax_cdf


def saveImage(img, ofn, qual=None):
    '''SAVEIMAGE - Save an image
    SAVEIMAGE(img, ofn) saves the image IMG to the file named OFN.
    Optional third argument specifies jpeg quality as a number between
    0 and 100, and must only be given if OFN ends in ".jpg". Default
    is 95.'''
    if qual is None:
        cv2.imwrite(ofn, img)
    else:
        cv2.imwrite(ofn, img, (cv2.IMWRITE_JPEG_QUALITY, qual))


if (len(sys.argv)<3):
  sys.stderr.write('\nUsage: %s image_file_glob output_dir \n' % (sys.argv[0]))
  sys.stderr.write('           Stretch contrast of a batch of images \n')
  sys.stderr.write('           Processed images are written to new files of same names in output_dir  \n\n')
  exit(1)

image_file_glob = sys.argv[1]
outdir = sys.argv[2]

image_files = sorted(glob.glob(image_file_glob))

for image_file in image_files:
  print("Equalizing: %s" % (image_file))
  img = cv2.imread(image_file, cv2.IMREAD_ANYDEPTH + cv2.IMREAD_GRAYSCALE)

#  img_eq = cv2.equalizeHist(img)

#  clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
  clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(32,32))
  img_eq = clahe.apply(img)
  
  # Contrast stretching
#  p2, p98 = np.percentile(img, (2, 98))
#  img_rescale = exposure.rescale_intensity(img, in_range=(p2, p98))

  basename = os.path.split(image_file)[-1]
  outname = os.path.join(outdir,basename)

  saveImage(img_eq, outname)

