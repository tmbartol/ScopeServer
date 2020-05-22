#!/usr/bin/env python3

import socket
import threading
import socketserver
import signal
import sys
import tty
import termios
import time
import datetime
import re
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import queue
import subprocess as sp
import picamera
from fractions import Fraction
import io
import base64
import cv2
import astropy
from astropy.stats import sigma_clipped_stats, mad_std
from astropy.visualization import SqrtStretch
from astropy.visualization.mpl_normalize import ImageNormalize
from photutils import  DAOStarFinder, CircularAperture
from photutils.psf.groupstars import DAOGroup

from PIL import Image
import random
import glob


# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# Signal handler for shutdown
def signal_handler(signal, frame):
  global fd, old_settings
  try:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  except:
    pass
  sys.stderr.write('\r\nAutoGuider caught signal %d\r\n' % (signal))
  sys.stderr.write('AutoGuider shutting down.\r\n')

  sys.exit(0)


# Get IP address
import netifaces as ni
def get_ip(iface = 'wlan0'):
  try:
    ni.ifaddresses(iface)
    ip = ni.ifaddresses(iface)[ni.AF_INET][0]['addr']
  except:
    ip = '127.0.0.1'
  return ip


# Class to get raw characters from terminal input
class _Getch:
  def __call__(self):
    global fd, old_settings
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
      if ch == '\x1b':
        ch = ch + sys.stdin.read(2)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):
  def handle(self):
    try:
      while ag.server_running:
      # Receive data from client
        data = self.request.recv(1024)
#        sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('ascii')))
        if data:
          # Process client command
          cmd = data.decode('utf-8').strip()
          if cmd:
            if cmd[0] == '{':
              cmd = cmd[1:-1]
              response = ag.process_autoguider_cmd(cmd)
            if response:
              # Send response to client
              response = response.encode('utf-8')
#              sys.stderr.write( 'Server responding:  %s\r\n' % (response) )
#              sys.stderr.write( 'Server sending:  %d bytes\r\n' % (len(response)) )
              if cmd == 'get_view':
                self.request.sendall( str(len(response)).encode('utf-8') )
              self.request.sendall(response)
        else:
          return
    except ConnectionResetError:
      sys.stderr.write('Connection reset by peer.\r\n')
      pass


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
  allow_reuse_address = True
  daemon_threads = True
  timeout = None
  request_queue_size = 20



class autoguider:

  def __init__(self, server_address, port):
    self.input_running = False
    self.server_running = False
    self.capture_continuous = False
    self.autoguider_q = queue.Queue(maxsize=0)
    self.autoguider_running = False

    self.autoguider_host = server_address
    self.autoguider_port = port
    host_dict = {}
    host_dict['10.0.1.23'] = '10.0.1.20'
    host_dict['192.168.50.10'] = '192.168.50.5'
    self.scopeserver_host = host_dict[self.autoguider_host]
    self.scopeserver_port = 54030


    self.init_cam()

    self.status = 'idle %d' % (self.img_count)


  def get_status(self):
    status = self.status
    return status


  def get_view(self):
    '''
    my_path = os.path.split(os.path.realpath(__file__))[0]
    img_path = os.path.join(my_path,'images_2')
    img_fns = sorted(glob.glob(img_path + '/*'))
    img_fn = random.choice(img_fns)
    img_buf = io.BytesIO()
    Image.open(img_fn).resize((400,300)).convert('L').save(img_buf,"JPEG")
    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')
    '''

#    sys.stderr.write('sending guider view:  %d bytes\r\n' % (len(self.guider_view)))

    return self.guider_view


  # Start server thread
  def server_start(self):
    self.input_start()
    self.autoguider_control_start()
    self.server_running = True

    self.threaded_server = ThreadedTCPServer((self.autoguider_host, self.autoguider_port), ThreadedTCPRequestHandler)

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    self.server_thread = threading.Thread(target=self.threaded_server.serve_forever)
    # Exit the server thread when the main thread terminates
    self.server_thread.daemon = True
    self.server_thread.start()

    sys.stderr.write('\r\nConnecting to ScopeServer at %s:%d ...\r\n' %(self.scopeserver_host,self.scopeserver_port))
    self.scopeserver_connect()

    self.autoguider_q.put(('sync_time', None))

    sys.stderr.write('\r\nWaiting for a command, type q to quit...\r\n')
    sys.stderr.write('   h -> print this help message\r\n')
    sys.stderr.write('   1 -> capture one image\r\n')
    sys.stderr.write('   c -> toggle continuous capture\r\n')
    sys.stderr.write('   f -> find guide star\r\n')
    sys.stderr.write('   g -> start guiding\r\n')
    sys.stderr.write('   s -> stop guiding\r\n')
    sys.stderr.write('   w -> wifi connect with ScopeServer\r\n')
    sys.stderr.write('   t -> synchronize time with ScopeServer\r\n')
    sys.stderr.write('   r -> reboot Autoguider rpi system\r\n')
    sys.stderr.write('   x -> shutdown Autoguider rpi system\r\n')
    sys.stderr.write('   q -> quit Autoguider app\r\n\r\n')

    self.server_thread.join()


  # Stop server thread
  def server_stop(self):
    if self.server_running:
      self.server_running=False
      sys.stderr.write('AutoGuider shutting down.\r\n')

      self.cam.close()
      self.threaded_server.shutdown()
      self.server_thread.join()


  # Start thread to get terminal input
  def input_start(self):
    self.input_running = True
    self.thread_get_input = threading.Thread(target=self.get_input)
    self.thread_get_input.setDaemon(1)
    self.thread_get_input.start()


  # Start thread for autoguider control
  def autoguider_control_start(self):
    self.thread_autoguider_control = threading.Thread(target=self.autoguider_control)
    self.thread_autoguider_control.setDaemon(1)
    self.thread_autoguider_control.start()


  def autoguider_control(self):
    sys.stderr.write('\r\nautoguider_control: thread starting...\r\n')
    while True:
      if self.autoguider_running:
        # if autoguiding in progress get next autoguiding command but do not wait if queue is empty
        try:
          autoguider_cmd = self.autoguider_q.get_nowait()
        except queue.Empty:
          autoguider_cmd = ('autoguider_continue', None)
      else:
        # if not autoguiding wait to receive autoguiding command
#        sys.stderr.write('autoguiding_control: idle, waiting for command\r\n')
        autoguider_cmd = self.autoguider_q.get()

      cmd = autoguider_cmd[0]
      cmd_arg = autoguider_cmd[1]

      if cmd == 'jog_pos':
        pass

      elif cmd == 'guide_start':
        sys.stderr.write('Guiding Started...\r\n')
        self.img_count = 0
        self.status = 'finding_guide_star %d' % (self.img_count)
        self.nightshot_sim()
        self.find_guide_star()
        self.status = 'found_guide_star %d' % (self.img_count)
        self.guiding = True
        self.autoguider_q.put(('guide_star_drift', None))

      elif cmd == 'guide_stop':
        sys.stderr.write('Guiding Stopped\r\n')
        self.status = 'idle %d' % (self.img_count)
        self.guiding = False

      elif cmd == 'find_guide_star':
        self.status = 'finding_guide_star %d' % (self.img_count)
        self.nightshot_sim()
        self.find_guide_star()
        self.status = 'found_guide_star %d' % (self.img_count)

      elif cmd == 'guide_star_drift':
        if self.guiding:
          self.status = 'active_guiding %d' % (self.img_count)
          self.nightshot_sim()
          self.guide_star_drift()
          self.scopeserver_guide_star_correction()
        else:
          self.status = 'idle %d' % (self.img_count)

      elif cmd == 'capture_one':
        self.nightshot_sim()
        if (self.capture_continuous):
          self.status = 'imaging %d' % (self.img_count)
          self.autoguider_q.put(('capture_one', None))

      elif cmd == 'ss_connect':
        self.scopeserver_connect()
        time.sleep(1)
        self.sync_time()

      elif cmd == 'sync_time':
        self.sync_time()

      elif cmd == 'reboot_autoguider':
        time.sleep(1)
        os.system('reboot')

      elif cmd == 'shutdown_autoguider':
        time.sleep(1)
        os.system('shutdown now')

      elif cmd == 'autoguider_continue':
        pass


      if not cmd == 'autoguider_continue':
        self.autoguider_q.task_done()


  def scopeserver_connect(self):
    HOST = self.scopeserver_host
    PORT = self.scopeserver_port
    cmd = '{autoguider_connect}'

    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      try:
        sock.connect((HOST, PORT))
        sock.sendall(bytes(cmd, 'utf-8'))
    
        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8')
      except:
        response = ''

#    sys.stderr.write('\r\nAuto Guider Sent:     %s\r\n' % (response))
#    sys.stderr.write('Auto Guider Received: %s\r\n' % (response))
    if response == 'autoguider_connected':
      self.scopeserver_connected = True
      sys.stderr.write('ScopeServer Connected\r\n')
    else:
      self.scopeserver_connected = False
      sys.stderr.write('ScopeServer Not Connected\r\n')


  # Get input from terminal
  def get_input(self):
    # Enter input loop
    while self.input_running:
      inkey = _Getch()
      while True:
        k=inkey()
        if k!='':
          break

      # 1, capture one image
      if k=='1':
        if (self.capture_continuous == False):
          self.autoguider_q.put(('capture_one', None))

      # c, toggle capture continuously
      elif k=='c':
        if (self.capture_continuous == False):
          self.capture_continuous = True
          self.autoguider_q.put(('capture_one', None))
        else:
          self.capture_continuous = False

      elif k=='f':
        self.autoguider_q.put(('find_guide_star', None))

      # g, start guiding
      elif k=='g':
        self.autoguider_q.put(('guide_start', None))

      # s, stop guiding
      elif k=='s':
        self.autoguider_q.put(('guide_stop', None))

      # Report Position
      elif k=='p':
        self.autoguider_q.put(('update_pos', True))
#        sys.stderr.write('  Current Pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # WiFi connect with ScopeServer
      elif k=='w':
        self.autoguider_q.put(('ss_connect', None))

      # Synchronize time with ScopeServer
      elif k=='t':
        self.autoguider_q.put(('sync_time', None))

      # Reboot Autoguider RPI System
      elif k=='r':
        self.autoguider_q.put(('reboot_autoguider', None))

      # Shutdown Autoguider RPI System
      elif k=='x':
        self.autoguider_q.put(('shutdown_autoguider', None))

      # Shutdown the server
      elif k=='q':
        self.input_running = False

      # Print help message
      elif k=='h':
        sys.stderr.write('\r\n>>> ' + str(k) + '\r\n')
        sys.stderr.write('   h -> print this help message\r\n')
        sys.stderr.write('   1 -> capture one image\r\n')
        sys.stderr.write('   c -> toggle continuous capture\r\n')
        sys.stderr.write('   f -> find guide star\r\n')
        sys.stderr.write('   g -> start guiding\r\n')
        sys.stderr.write('   s -> stop guiding\r\n')
        sys.stderr.write('   w -> wifi connect with ScopeServer\r\n')
        sys.stderr.write('   t -> synchronize time with ScopeServer\r\n')
        sys.stderr.write('   r -> reboot Autoguider rpi system\r\n')
        sys.stderr.write('   x -> shutdown Autoguider rpi system\r\n')
        sys.stderr.write('   q -> quit and shutdown Autoguider\r\n')
      else:
        sys.stderr.write('>>> ' + str(k) + ' \r\n')
        sys.stderr.write('Unknown key! Please type q to quit.\r\n')
        sys.stderr.write('   h -> print this help message\r\n')
        sys.stderr.write('   1 -> capture one image\r\n')
        sys.stderr.write('   c -> toggle continuous capture\r\n')
        sys.stderr.write('   f -> find guide star\r\n')
        sys.stderr.write('   g -> start guiding\r\n')
        sys.stderr.write('   s -> stop guiding\r\n')
        sys.stderr.write('   w -> wifi connect with ScopeServer\r\n')
        sys.stderr.write('   t -> synchronize time with ScopeServer\r\n')
        sys.stderr.write('   r -> reboot Autoguider rpi system\r\n')
        sys.stderr.write('   x -> shutdown Autoguider rpi system\r\n')
        sys.stderr.write('   q -> quit and shutdown Autoguider\r\n')
    self.server_stop()


  # Process BlueShift Telescope Auto_Guider Protocol Commands
  def process_autoguider_cmd(self,cmd):

    response = None

    if cmd.split()[0] == 'get_autoguider_time':
      response = 'autoguider_time %s' % (str(time.time_ns()))

    elif cmd == 'scopeserver_connect':
      self.scopeserver_connected = True
      response = 'scopeserver_connected'
      self.autoguider_q.put(('sync_time', None))
      sys.stderr.write('ScopeServer Connected\r\n')

    # toggle capture continuously
    elif cmd == 'toggle_imaging':
      if not self.capture_continuous:
        self.capture_continuous = True
        self.autoguider_q.put(('capture_one', None))
      else:
        self.status = 'idle %d' % (self.img_count)
        self.capture_continuous = False
      response = 'ack_toggle_imaging'

    elif cmd == 'find_guide_star':
      self.autoguider_q.put(('find_guide_star', None))
      response = 'ack_find_guide_star'

    elif cmd == 'center_guide_star':
#      self.autoguider_q.put(('center_guide_star', None))
      response = 'ack_center_guide_star'

    elif cmd == 'toggle_guiding':
      if not self.guiding:
        self.autoguider_q.put(('guide_start', None))
      else:
        self.autoguider_q.put(('guide_stop', None))
      response = 'ack_toggle_guiding'

    elif cmd == 'guide_start':
      self.autoguider_q.put(('guide_start', None))
      response = 'guiding_started'

    elif cmd == 'guide_stop':
      self.autoguider_q.put(('guide_stop', None))
      response = 'ack_guiding_stopped'

    elif cmd == 'guide_star_drift':
      self.autoguider_q.put(('guide_star_drift', None))
      response = 'ack_guide_star_drift'

    elif cmd == 'get_status':
      response = str(self.get_status())

    elif cmd == 'get_view':
      response = str(self.get_view())

    elif cmd == 'reset_server':
      pass

    elif cmd == 'quit_server':
      self.server_stop()

    elif cmd == 'reboot_autoguider':
      response = 'reboot_autoguider'
      self.autoguider_q.put(('reboot_autoguider', None))

    elif cmd == 'shutdown_autoguider':
      response = 'shutdown_autoguider'
      self.autoguider_q.put(('shutdown_autoguider', None))

    else:
      pass

    return response


  def init_cam(self):
    sys.stderr.write('\r\n')
    sys.stderr.write('Configuring Camera...\r\n')

    self.sensor_dimx = 3.68
    self.sensor_dimy = 2.76
    self.npix_x = 3280
    self.npix_y = 2464
    self.arcsec_per_pixel = (3600/self.npix_x)*2*np.arctan(self.sensor_dimx/(2*240))*180/np.pi
    self.ra_counts_per_arcsec=40000*576/(360*3600.)
    self.dec_counts_per_arcsec=40000*450/(360*3600.)
    self.ra_counts_per_pixel = self.ra_counts_per_arcsec*self.arcsec_per_pixel
    self.dec_counts_per_pixel = self.dec_counts_per_arcsec*self.arcsec_per_pixel

    self.cam = picamera.PiCamera()
    self.cam.sensor_mode = 2
    self.cam.resolution = (3280,2464)
    self.cam.rotation = 180

    self.exposure = 1.0

    self.cam.framerate = 1/self.exposure
    self.cam.shutter_speed = int(self.exposure*1e6/1.0)
    self.cam.iso = 800
    self.cam.meter_mode = 'average'
    self.cam.exposure_mode = 'night'
    self.cam.image_denoise = False
    time.sleep(5)
    self.cam.exposure_mode = 'off'

#    self.cam.capture('./init_image.jpg', use_video_port=True, quality=75)
    # Capture to memory buffer:
    img_buf = io.BytesIO()
    sys.stderr.write('Capturing test image to buffer  \r\n')
    self.cam.capture(img_buf, use_video_port=True, format='jpeg', quality=75)
    img_buf.truncate()
    img_buf.seek(0)
    img_data = np.frombuffer(img_buf.getvalue(), dtype=np.uint8)
    img = cv2.imdecode(img_data, cv2.IMREAD_GRAYSCALE)
    img_buf.seek(0)
    img_buf.truncate()
    sys.stderr.write('  Captured: %s\r\n' % (str(img.shape)))

    sys.stderr.write('Camera ready:\r\n')
    sys.stderr.write('  Exposure: %s\r\n' % (str(self.cam.exposure_speed)))
    sys.stderr.write('  ISO: %s\r\n' % (str(self.cam.iso)))
    sys.stderr.write('  Digital gain: %s\r\n' % (str(float(self.cam.digital_gain))))
    sys.stderr.write('  Analog gain: %s\r\n' % (str(float(self.cam.analog_gain))))

    self.img_count = 0
    self.guider_view = 'data:image/jpg;base64,'
    self.image = None
    self.guide_star_rect = None


  def nightshot(self):

    # Capture to memory buffer:
    img_buf = io.BytesIO()
    sys.stderr.write('  Capturing image to buffer  \r\n')
    self.cam.capture(img_buf, use_video_port=True, format='jpeg', quality=75)
    img_buf.truncate()
    img_buf.seek(0)
    img_data = np.frombuffer(img_buf.getvalue(), dtype=np.uint8)
#    img_buf.seek(0)
#    img_buf.truncate()
    img = cv2.imdecode(img_data, cv2.IMREAD_GRAYSCALE)
    img = np.flipud(img)
    self.image = img
    img = cv2.resize(img,(400,300))
    img_buf = io.BytesIO(cv2.imencode(".jpg", img, (cv2.IMWRITE_JPEG_QUALITY, 75))[1])
    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')

#    ofn = './images/image.jpg'
#    cv2.imwrite(ofn, img, (cv2.IMWRITE_JPEG_QUALITY, 75))
    sys.stderr.write('      Captured: %s\r\n' % (str(img.shape)))

    self.img_count += 1

    '''
    # Capture to file:
    imfn = './images/image_seq_%04d.%s.jpg' % (self.img_count,str(time.time()))
    sys.stderr.write('  Capturing image: %s  \r\n' % (imfn))
    
    self.cam.capture(imfn, use_video_port=True, quality=75)
#    self.cam.capture(img,format='bgr')
    sys.stderr.write('      Done\r\n')

    #self.cam.capture_sequence(['./images/image%02d.jpg' % i for i in range(5)],quality=25)
    '''



  def nightshot_sim(self):

    time.sleep(1.2)
    img_dir = './images_sim/'
    img_fns = sorted(glob.glob(img_dir + '*'))
    i = self.img_count % len(img_fns)
    img_fn = img_fns[i]
    sys.stderr.write('Reading sim image: %s\r\n' % (img_fn))

    img = cv2.imread(img_fn, cv2.IMREAD_GRAYSCALE)
    img = np.flipud(img)
    self.image = img

#    img = cv2.resize(img,(400,300))
#    img_buf = io.BytesIO(cv2.imencode(".jpg", img, (cv2.IMWRITE_JPEG_QUALITY, 75))[1])
#    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')

    self.plot_guider_view()

    self.img_count += 1


  def plot_guider_view(self):
    img = self.image
    plt.style.use('dark_background')
    fig,ax = plt.subplots(1)
    ax.spines['bottom'].set_color('#d00000')
    ax.spines['top'].set_color('#d00000') 
    ax.spines['right'].set_color('#d00000')
    ax.spines['left'].set_color('#d00000')
    ax.tick_params(axis='x', colors='#d00000', which='both')
    ax.tick_params(axis='y', colors='#d00000', which='both')
    ax.yaxis.label.set_color('#d00000')
    ax.xaxis.label.set_color('#d00000')
    ax.title.set_color('#d00000')

    norm = ImageNormalize(stretch=SqrtStretch())
    ax.imshow(255-img, cmap='Greys', origin='lower', norm=norm)

    if type(self.guide_star_rect) != type(None):
      rect = patches.Rectangle(self.guide_star_rect[0],self.guide_star_rect[1],self.guide_star_rect[2],linewidth=1,edgecolor='r',facecolor='none')
      ax.add_patch(rect)

    ax.text(0.05, 0.90, str(self.img_count),
        verticalalignment='bottom', horizontalalignment='left',
        transform=ax.transAxes,
        color='#d00000', fontsize=12)

    img_buf = io.BytesIO()
    plt.savefig(img_buf, format='png')
    plt.close()
    self.guider_view = 'data:image/png;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')


  def find_guide_star(self):

    sys.stderr.write('Finding Guide Star...\r\n')
    self.ref_image = self.image
    img = self.ref_image

    w = 128
    h = 128

    mean, median, std = sigma_clipped_stats(img, sigma=3.0)
    bkg_sigma = mad_std(img)
#    print((mean, median, std))
#    print(bkg_sigma)

    daofind = DAOStarFinder(fwhm=9.0, sharphi=0.65, threshold=20.0*bkg_sigma, exclude_border=True)

    sources = daofind(img - median)
    for col in sources.colnames:
        sources[col].info.format = '%.8g'  # for consistent table output

    sources.sort(['peak'])
    sources.reverse()

#    print('')
#    print('All sources:')
#    print(sources)

#    print('')
#    print('Guide Source:')
#    print(sources[0])

    self.guide_star_pos = np.array([sources['xcentroid'][0],sources['ycentroid'][0]])
    self.guide_star_rect = np.array([self.guide_star_pos-[w/2.,h/2.],w,h])

    swim_dir = './swim_tmp/'
    img_ref_fn = swim_dir + 'guide_ref.jpg'
    self.image_crop_rect(img, self.guide_star_rect, img_ref_fn)

    # Show image with identified sources
    plt.style.use('dark_background')
    fig,ax = plt.subplots(1)
    ax.spines['bottom'].set_color('#d00000')
    ax.spines['top'].set_color('#d00000') 
    ax.spines['right'].set_color('#d00000')
    ax.spines['left'].set_color('#d00000')
    ax.tick_params(axis='x', colors='#d00000', which='both')
    ax.tick_params(axis='y', colors='#d00000', which='both')
    ax.yaxis.label.set_color('#d00000')
    ax.xaxis.label.set_color('#d00000')
    ax.title.set_color('#d00000')

    norm = ImageNormalize(stretch=SqrtStretch())
    ax.imshow(255-img, cmap='Greys', origin='lower', norm=norm)

    rect = patches.Rectangle(self.guide_star_rect[0],self.guide_star_rect[1],self.guide_star_rect[2],linewidth=1,edgecolor='r',facecolor='none')
    ax.add_patch(rect)

    ax.text(0.05, 0.90, 'Guide Star',
        verticalalignment='bottom', horizontalalignment='left',
        transform=ax.transAxes,
        color='#d00000', fontsize=12)

    positions = np.transpose((sources['xcentroid'], sources['ycentroid']))
    apertures = CircularAperture(positions, r=5.)
    apertures.plot(color='blue', lw=1.5, alpha=0.5)
    img_buf = io.BytesIO()
    plt.savefig(img_buf, format='png')
    plt.close()
    self.guider_view = 'data:image/png;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')
    sys.stderr.write('Found Guide Star\r\n')


  def image_crop_rect(self, img, rect, img_out_fn):
    x = int(rect[0][0])
    y = int(rect[0][1])
    w = int(rect[1])
    h = int(rect[2])
    img_cropped = img[y:(y+h), x:(x+w)]
    cv2.imwrite(img_out_fn, img_cropped)


  def guide_star_drift(self):

    test_img_dir = './images_2/'
    test_img_fns = sorted(glob.glob(test_img_dir + '*'))

    swim_dir = './swim_tmp/'
    img_ref_fn = swim_dir + 'guide_ref.jpg'
    img_update_fn = swim_dir + 'guide_update.jpg'

    guide_star_dat = np.empty((0,5))
    guide_star_dat = np.append(guide_star_dat,[0,0,0,0,0]).reshape(-1,5)

    dx = 0
    dy = 0
    w = 128
    h = 128

    guide_update_rect = np.array([self.guide_star_pos+[dx,dy]-[w/2.,h/2.],w,h])
    self.image_crop_rect(self.image, guide_update_rect, img_update_fn)

    swim_cmd_template = '/home/pi/bin/swim 128 -i 2 -w -0.61 %s %s'
    swim_cmd = swim_cmd_template % (img_ref_fn, img_update_fn)
#    sys.stderr.write('%s\r\n' % (swim_cmd))
    swim_log = sp.run([swim_cmd],shell=True,stdout=sp.PIPE,stderr=sp.PIPE)
    swim_stdout = swim_log.stdout.decode('utf-8')
    swim_stderr = swim_log.stdout.decode('utf-8')
    toks = swim_stdout.split()
#    print('swim output: \n  ' + swim_stdout)
    snr = float(toks[0][0:-1])
    dxi = float(toks[8][1:])
    dyi = float(toks[9])
    dx += dxi
    dy += dyi
    self.dRA = int(dxi*self.ra_counts_per_pixel)
    self.dDEC = int(dyi*self.dec_counts_per_pixel)

#    sys.stderr.write('swim match:  SNR: %g  dX: %g  dY: %g\r\n' % (snr, dxi, dyi))
    sys.stderr.write('swim match:  SNR: %g  dRA: %d  dDEC: %d\r\n' % (snr, self.dRA, self.dDEC))

#    guide_star_dat = np.append(guide_star_dat,[t,dx,dy,dxi,dyi]).reshape(-1,5)


  def scopeserver_guide_star_correction(self):
    HOST = self.scopeserver_host
    PORT = self.scopeserver_port
    cmd = '{guide_star_correction %d %d}' % (self.dRA, self.dDEC)

#    sys.stderr.write('\r\nAuto Guider Sending:    %s\r\n' % (cmd))
    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      try:
        sock.settimeout(1.0)
        sock.connect((HOST, PORT))
        sock.settimeout(None)
        sock.sendall(bytes(cmd, 'utf-8'))
    
        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8')
      except:
        response = ''

#    sys.stderr.write('Auto Guider Sent:       %s\r\n' % (cmd))
#    sys.stderr.write('Auto Guider Received:   %s\r\n' % (response))
    if response == 'ack_guide_star_correction':
      self.scopeserver_connected = True
    else:
      self.scopeserver_connected = False
      sys.stderr.write('ScopeServer Not Connected\r\n')


  def sync_time(self):

    if not self.scopeserver_connected:
      sys.stderr.write('Could Not Synchronize Time\r\n')
      sys.stderr.write('ScopeServer Not Connected\r\n')
      return

    HOST = self.scopeserver_host
    PORT = self.scopeserver_port

    cmd = '{get_scopeserver_time}'

    dt_array = np.empty((0),'int64')
    t_diff_array = np.empty((0),'int64')

    n = 50
    for i in range(n):
      # Create a socket (SOCK_STREAM means a TCP socket)
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        # Connect to server and send cmd
        sock.connect((HOST, PORT))
        t0 = time.time_ns()
        sock.sendall(bytes(cmd, 'utf-8'))

        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8')
        t2 = time.time_ns()

      if response.split()[0] == 'scopeserver_time':
        t_ss = int(response.split()[1])
        dt = t2 - t0
        t_diff = (t_ss - dt/2) - t0
        dt_array = np.append(dt_array,dt)
        t_diff_array = np.append(t_diff_array,t_diff)
      else:
        self.scopeserver_connected = False
        sys.stderr.write('Could Not Synchronize Time\r\n')
        sys.stderr.write('ScopeServer Not Connected\r\n')

    dt = dt_array*1e-9
    t_diff = t_diff_array*1e-9
    t_diff_mu = int(t_diff_array.mean())
    sys.stderr.write('\r\ndt = %.4g (min:%.4g max:%.4g stddev:+-%.4g)\r\n' % (dt.mean(),dt.min(),dt.max(),dt.std()))
    sys.stderr.write('t_diff = %.4g (min:%.4g max:%.4g stddev:+-%.4g)\r\n' % (t_diff.mean(),t_diff.min(),t_diff.max(),t_diff.std()))
    time.clock_settime_ns(time.CLOCK_REALTIME,time.time_ns()+t_diff_mu)
    sys.stderr.write('Time Synchronized\r\n')
    sys.stderr.write('Current Time:  %s\r\n' % (time.strftime('%a %b %d %H:%M:%S %Z %Y')))




if (__name__ == '__main__'):

#  if (len(sys.argv)<3):
#    print('\nUsage: %s server_address port\n' % (sys.argv[0]))
#    print('   Example: %s 10.0.1.15 4030\n' % (sys.argv[0]))
#    sys.exit()
#  server_address = sys.argv[1]
#  port = int(sys.argv[2])

  pid = os.getpid()
  try:
    pidfile = open('/var/run/picam_autoguider.pid','w')
    pidfile.write('%d\n' % (pid))
    pidfile.close()
  except:
    pass

  server_address = get_ip()
  port = 54040

  sys.stderr.write('\r\nStarting PiCAM AutoGuider at %s:%d\r\n' % (server_address, port))

  ag = autoguider(server_address, port)
  ag.server_start()

