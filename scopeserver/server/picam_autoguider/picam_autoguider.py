#!/usr/bin/env python3

import socket
import threading
import socketserver
import signal
import sys
import gc
import tty
import termios
import time
import datetime
import re
import math
import numpy as np
import scipy.stats as sps
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import queue
import subprocess as sp
import picamerax as picamera
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
import skimage.exposure as skie
import skimage.morphology as morph

from PIL import Image, ImageFont, ImageDraw
import random
import glob

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning) 
warnings.filterwarnings("ignore", category=FutureWarning) 
warnings.filterwarnings("ignore")


# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# Signal handler for shutdown
def signal_handler(signal, frame):
  global fd, old_settings
  try:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  except:
    pass
  sys.stderr.write('\r\nAutoGuider caught signal %d\r\n' % (signal))
  sys.stderr.write('AutoGuider shutting down...\r\n')

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
      # Receive data from client
      data = self.request.recv(1024)
#      sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('ascii')))
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
#            sys.stderr.write( 'Server responding:  %s\r\n' % (response) )
#            sys.stderr.write( 'Server sending:  %d bytes\r\n' % (len(response)) )
            if cmd == 'get_view':
#              sys.stderr.write( 'get_view at: %d  len: %d\r\n' % (int(time.time()), len(response)) )
              self.request.sendall( str(len(response)).encode('utf-8') )
            self.request.sendall(response)
            if cmd == 'get_view':
              ack = self.request.recv(1024).decode('utf-8')
              if ack == 'ack':
#                sys.stderr.write('  \r\ncmd:  %s  got ack\r\n' % (str(cmd)))
                pass
              else:
#                sys.stderr.write('  \r\ncmd:  %s  no ack\r\n' % (str(cmd)))
                pass
            elif cmd == 'scopeserver_status':
              status = self.request.recv(1024).decode('utf-8')
              if status:
                status = eval(status)
                self.request.sendall(b'ack')
                meridian_mode = int(status['meridian_mode'])
                ag.autoguider_q.put(('update_meridian_mode', meridian_mode))
#                sys.stderr.write('1) setting meridian_mode: %d\r\n' % (meridian_mode))
    except Exception as e:
      '''
      sys.stderr.write('Exception in Autoguider TCP Command Processor.\r\n')
      sys.stderr.write('  cmd:  %s\r\n' % (str(cmd)))
      if hasattr(e, 'message'):
        sys.stderr.write('    %s\r\n' % (str(e.message)))
      else:
        sys.stderr.write('    %s\r\n' % (str(e)))
      '''
      pass
    finally:
      self.request.close()


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
  allow_reuse_address = True
#  daemon_threads = True
  daemon_threads = False
  timeout = None
  request_queue_size = 100


def to_precision(v,p):
  fstr = '%%.%dg' % (p)
  vf = fstr % (abs(v))
  se = vf.split('e')
  mant = se[0]
  if len(se) > 1:
    expon = 'e'+ se[1]
  else:
    expon = ''
  nz = ''
  sd = mant.split('.')
  if len(sd) > 1:
    if sd[0] == '0':
      if len(mant) < p+2:
        nz = '0'*(p+2 - len(mant))
    else:
      if len(mant) < p+1:
        nz = '0'*(p+1 - len(mant))
  else:
    if len(mant) < p:
      if sd[0] == '0':
        nz = '.' + '0'*(p+1 - len(mant))
      else:
        nz = '.' + '0'*(p - len(mant))
  if (v < 0.0):
      mant = '-' + mant
  vfinal = mant + nz + expon
  return(vfinal)



class autoguider:

  def __init__(self, server_address, port, sim_img_mode=False):
    self.verbose = True
    self.dir_path = os.path.dirname(os.path.realpath(__file__))
    self.input_running = False
    self.server_running = False
    self.capture_continuous = False
    self.autoguider_q = queue.Queue(maxsize=0)
    self.autoguider_running = False
    self.guiding = False
    self.analyzing = False
    self.gcorr_plot = False
    self.correction_log = None
    self.sim_img_mode=sim_img_mode
    self.dRA = 0
    self.dDEC = 0
    self.driftRA = 0
    self.driftDEC = 0
    self.capture_time = 0
    self.prev_capture_time = 0
    self.drift_ana = np.empty((0,4),'float')
    self.drift_corr = np.empty((0,3),'float')
    self.pos_ra = 0
    self.pos_dec = 0
    self.pec_idx = 0

    self.autoguider_host = server_address
    self.autoguider_port = port
    host_dict = {}
    host_dict['10.0.1.14'] = '10.0.1.13'
    host_dict['10.0.1.23'] = '10.0.1.20'
    host_dict['10.0.1.24'] = '10.0.1.23'
    host_dict['192.168.50.10'] = '192.168.50.5'
    self.scopeserver_host = host_dict[self.autoguider_host]
    self.scopeserver_port = 54030

    self.status = 'idle'
    self.gamma = 1.0
    self.blackpoint = 0
    self.mag = 1
    self.view_scale = 1.0
    self.img_pick = False
    self.pick_pnt_1 = None
    self.pick_pnt_2 = None
#    self.blackpoint = 20
    self.exposure = 1.0  # exposure time in seconds
    self.autoguider_interval = 1.0
    self.set_gamma_lut(gamma=self.gamma, blackpoint=self.blackpoint)
    self.meridian_mode = 1
    self.cam = None
    self.init_cam()


  def get_status(self):
    status_str = '%s %d %g %d %d %g %g' % (self.status, self.img_count, self.gamma, self.blackpoint, self.mag, self.exposure, self.autoguider_interval)
    return status_str


  def get_view(self):

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
    self.autoguider_q.put(('get_scopeserver_status', None))

    sys.stderr.write('\r\nWaiting for a command, type q to quit...\r\n')
    sys.stderr.write('   h -> print this help message\r\n')
    sys.stderr.write('   1 -> capture one image\r\n')
    sys.stderr.write('   c -> toggle continuous capture\r\n')
    sys.stderr.write('   f -> find guide star\r\n')
    sys.stderr.write('   g -> start guiding\r\n')
    sys.stderr.write('   s -> stop guiding\r\n')
    sys.stderr.write('   a -> toggle drift analysis\r\n')
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
      sys.stderr.write('AutoGuider shutting down...\r\n')

      if self.cam != None:
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

      self.cancel_capture_list = ['find_guide_star','guide_start','guide_star_drift','analysis_start','guide_star_drift_analysis']

      # Cancel continuous capture as appropriate
      if cmd in self.cancel_capture_list:
        self.status = 'idle'
        self.capture_continuous = False

      '''
      # Cancel guiding as appropriate
      if cmd != 'guide_star_drift' and cmd != 'find_guide_star':
        self.status = 'idle'
        self.guiding = False

      # Cancel drift analysis as appropriate
      if cmd != 'guide_star_drift_analysis' and cmd != 'find_guide_star':
        self.status = 'idle'
        self.analyzing = False
      '''

      if cmd == 'set_gamma_lut':
        self.set_gamma_lut(gamma=self.gamma, blackpoint=self.blackpoint)
        self.make_guider_view()

      elif cmd == 'set_exposure':
        self.status = 'init_cam'
        if self.cam != None:
          self.cam.close()
        self.init_cam()

      elif cmd == 'set_mag':
        self.make_guider_view()

      elif cmd == 'set_img_pick':
        self.img_pick = True
        pick_x = cmd_arg[0]
        pick_y = cmd_arg[1]
        if type(self.pick_pnt_1) == type(None):
          self.pick_pnt_1 = np.array((pick_x, pick_y, self.view_scale))
        elif type(self.pick_pnt_2) == type(None):
          self.pick_pnt_2 = np.array((pick_x, pick_y, self.view_scale))
        else:
          self.pick_pnt_1 = self.pick_pnt_2
          self.pick_pnt_2 = np.array((pick_x, pick_y, self.view_scale))
#        sys.stderr.write('  pick_pnt_1:  %s\r\n' % (str(self.pick_pnt_1)))
#        sys.stderr.write('  pick_pnt_2:  %s\r\n' % (str(self.pick_pnt_2)))
        self.make_guider_view()

      elif cmd == 'clear_img_pick':
        self.img_pick = False
        self.pick_pnt_1 = None
        self.pick_pnt_2 = None
        self.make_guider_view()

      elif cmd == 'guide_start':
        self.status = 'finding_guide_star'
        if self.verbose:
          sys.stderr.write('Guiding Started...\r\n')
        self.img_count = 0
        self.drift_corr = np.empty((0,3),'float')
        if self.sim_img_mode:
          self.nightshot_sim()
        else:
          self.nightshot()
        self.find_guide_star()
        if self.status == 'found_guide_star':
          self.guiding = True
          logfn = os.path.join(self.dir_path,'correction_log.dat')
          self.correction_log = open(logfn,'w')
          self.drift_t0 = time.time_ns()
          self.autoguider_q.put(('guide_star_drift', None))
        else:
          self.autoguider_q.put(('guide_stop', None))

      elif cmd == 'guide_stop':
        self.status = 'idle'
        if self.verbose:
          sys.stderr.write('Guiding Stopped\r\n')
        self.guiding = False
        if self.correction_log:
          self.correction_log.close()
          self.correction_log = None

      elif cmd == 'find_guide_star':
        self.status = 'finding_guide_star'
        if self.sim_img_mode:
          self.nightshot_sim()
        else:
          self.nightshot()
        self.find_guide_star()

      elif cmd == 'guide_star_drift':
        if self.guiding:
          self.status = 'active_guiding'
          if self.sim_img_mode:
            self.nightshot_sim()
          else:
            self.nightshot()
          self.guide_star_drift()
          self.scopeserver_guide_star_correction()
        else:
          self.status = 'idle'

      elif cmd == 'move1to2':
        moveRA = -(self.pick_pnt_2[0] - self.pick_pnt_1[0])/self.pick_pnt_1[2]
        moveDEC = -(self.pick_pnt_2[1] - self.pick_pnt_1[1])/self.pick_pnt_1[2]
        self.dRA = int(moveRA*self.ra_counts_per_pixel)
        self.dDEC = int(moveDEC*self.dec_counts_per_pixel)
        self.scopeserver_guide_star_correction(reset_target=True)

      elif cmd == 'center_guide_star':
        moveRA = -(800 - self.pick_pnt_1[0])/self.pick_pnt_1[2]
        moveDEC = -(600 - self.pick_pnt_1[1])/self.pick_pnt_1[2]
        self.dRA = int(moveRA*self.ra_counts_per_pixel)
        self.dDEC = int(moveDEC*self.dec_counts_per_pixel)
        self.scopeserver_guide_star_correction(reset_target=True)

      elif cmd == 'analysis_start':
        self.status = 'finding_guide_star'
        if self.verbose:
          sys.stderr.write('Drift Analysis Started...\r\n')
        self.img_count = 0
        self.driftRA = 0
        self.driftDEC = 0
        self.drift_ana = np.empty((0,4),'float')
        if self.sim_img_mode:
          self.nightshot_sim()
        else:
          self.nightshot()
        self.find_guide_star()
        if self.status == 'found_guide_star':
#          self.drift_ana = np.append(self.drift_ana, np.array([[0.,0.,0.,self.pec_idx]]), axis=0)
          self.analyzing = True
          self.autoguider_q.put(('get_scopeserver_status', None))
          self.autoguider_q.put(('guide_star_drift_analysis', None))
        else:
          self.autoguider_q.put(('analysis_stop', None))

      elif cmd == 'analysis_stop':
        self.status = 'idle'

        # save drift_analysis data
        t = self.drift_ana[1:,0]
        ra = self.drift_ana[1:,1]
        dec = self.drift_ana[1:,2]
        pex_idx = self.drift_ana[1:,3]
        outfile = open('/home/pi/src/scopeserver-git/scopeserver/server/picam_autoguider/drift_analysis_data/drift_analysis.dat','w')
        for i in range(len(t)):
          outfile.write('%g %g %g %g\n' % (t[i],ra[i],dec[i],pex_idx[i]))
        outfile.close()

        if self.verbose:
          sys.stderr.write('Drift Analysis Stopped\r\n')
        self.analyzing = False

      elif cmd == 'guide_star_drift_analysis':
        if self.analyzing:
          self.status = 'active_analyzing'
          if self.sim_img_mode:
            self.nightshot_sim()
          else:
            self.nightshot()
          self.guide_star_drift_analysis()
          self.autoguider_q.put(('get_scopeserver_status', None))
          self.autoguider_q.put(('guide_star_drift_analysis', None))
        else:
          self.status = 'idle'

      elif cmd == 'capture_one':
        if self.sim_img_mode:
          self.nightshot_sim()
        else:
          self.nightshot()
        self.make_guider_view()
        if (self.capture_continuous):
          self.status = 'imaging'
          self.autoguider_q.put(('capture_one', None))
        else:
          self.status = 'idle'

      elif cmd == 'ss_connect':
        self.scopeserver_connect()
        time.sleep(1)
        self.sync_time()

      elif cmd == 'sync_time':
        self.sync_time()

      elif cmd == 'get_scopeserver_status':
        self.get_scopeserver_status()

      elif cmd == 'update_meridian_mode':
        if cmd_arg != self.meridian_mode:
          self.meridian_mode = cmd_arg
#          if type(self.image_c) != type(None):
          if type(self.image) != type(None):
            self.image = cv2.rotate(self.image, cv2.ROTATE_180)
          self.make_guider_view()

      elif cmd == 'reset_autoguider':
        sys.stderr.write('\r\nResetting Autoguider...\r\n')
        time.sleep(1)
        os.system('sudo systemctl restart autoguider_control.service')
#        os.execl('/home/pi/src/scopeserver-git/scopeserver/server/picam_autoguider/picam_autoguider.py',(' '))
#        os.system('nohup sudo /home/pi/src/scopeserver-git/scopeserver/scopeserver_control_agreset.sh &')

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

    sys.stderr.write('Autoguider Requesting to Connect to ScopeServer...\r\n')
    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      try:
        sock.settimeout(0.1)
        sock.connect((HOST, PORT))
        sock.settimeout(None)
        sock.sendall(bytes(cmd, 'utf-8'))
    
        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8')
      except:
        response = ''
      finally:
        sock.close()

#    sys.stderr.write('\r\nAuto Guider Sent:     %s\r\n' % (response))
#    sys.stderr.write('Auto Guider Received: %s\r\n' % (response))
    if response == 'ack':
      self.scopeserver_connected = True
      sys.stderr.write('  ScopeServer Connected.\r\n')
    else:
      self.scopeserver_connected = False
      sys.stderr.write('  ScopeServer Not Connected.\r\n')


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

      # a, toggle drift analysis
      elif k=='a':
        if self.guiding:
          self.autoguider_q.put(('guide_stop', None))
        if not self.analyzing:
          self.autoguider_q.put(('analysis_start', None))
        else:
          self.autoguider_q.put(('analysis_stop', None))

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
        sys.stderr.write('   a -> toggle drift analysis\r\n')
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
        sys.stderr.write('   a -> toggle drift analysis\r\n')
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
      sys.stderr.write('ScopeServer Requesting to Connect to Autoguider...\r\n')
      self.scopeserver_connected = True
      self.autoguider_q.put(('sync_time', None))
      self.autoguider_q.put(('get_scopeserver_status', None))
      sys.stderr.write('  ScopeServer Connected.\r\n')
      response = 'ack'

    # toggle capture continuously
    elif cmd == 'toggle_imaging':
      if not self.capture_continuous:
        self.capture_continuous = True
        self.autoguider_q.put(('capture_one', None))
      else:
        self.status = 'idle'
        self.capture_continuous = False
      response = 'ack'

    elif cmd.split()[0] == 'set_img_pick':
      pick_x = int(4*float(cmd.split()[1]))
      pick_y = int(4*float(cmd.split()[2]))
      self.autoguider_q.put(('set_img_pick', (pick_x, pick_y)))
      response = 'ack'

    elif cmd.split()[0] == 'clear_img_pick':
      self.autoguider_q.put(('clear_img_pick', None))
      response = 'ack'

    elif cmd.split()[0] == 'move1to2':
      if (type(self.pick_pnt_1) != type(None)) and (type(self.pick_pnt_2) != type(None)):
        self.autoguider_q.put(('move1to2', None))
      response = 'ack'

    elif cmd.split()[0] == 'set_gamma_val':
      self.gamma = float(cmd.split()[1])
      self.autoguider_q.put(('set_gamma_lut', None))
      response = 'ack'

    elif cmd.split()[0] == 'set_bp_val':
      self.blackpoint = int(float(cmd.split()[1]))
      self.autoguider_q.put(('set_gamma_lut', None))
      response = 'ack'

    elif cmd.split()[0] == 'set_mag_val':
      self.mag = int(float(cmd.split()[1]))
      self.autoguider_q.put(('set_mag', None))
      response = 'ack'

    elif cmd.split()[0] == 'set_exposure_val':
      self.exposure = float(cmd.split()[1])
      self.autoguider_q.put(('set_exposure', None))
      response = 'ack'

    elif cmd.split()[0] == 'set_autoguider_interval_val':
      self.autoguider_interval = float(cmd.split()[1])
      response = 'ack'

    elif cmd == 'find_guide_star':
      self.autoguider_q.put(('find_guide_star', None))
      response = 'ack'

    elif cmd == 'center_guide_star':
      if (type(self.pick_pnt_1) != type(None)):
        self.autoguider_q.put(('center_guide_star', None))
      response = 'ack'

    elif cmd == 'toggle_guiding':
      if self.analyzing:
        self.autoguider_q.put(('analysis_stop', None))
      if not self.guiding:
        self.autoguider_q.put(('guide_start', None))
      else:
        self.autoguider_q.put(('guide_stop', None))
      response = 'ack'

    elif cmd == 'guide_start':
      self.autoguider_q.put(('guide_start', None))
      response = 'ack'

    elif cmd == 'guide_stop':
      self.autoguider_q.put(('guide_stop', None))
      response = 'ack'

    elif cmd == 'guide_star_drift':
      if self.guiding:
        self.autoguider_q.put(('guide_star_drift', None))
      response = 'ack'

    elif cmd == 'toggle_gcorr':
      self.gcorr_plot = not self.gcorr_plot
#      sys.stderr.write('Toggling Guide Correction Plot %s\r\n' % (str(self.gcorr_plot)))
      response = 'ack'

    elif cmd == 'get_status':
      response = str(self.get_status())

    elif cmd == 'toggle_analysis':
      if self.guiding:
        self.autoguider_q.put(('guide_stop', None))
      if not self.analyzing:
        self.autoguider_q.put(('analysis_start', None))
      else:
        self.autoguider_q.put(('analysis_stop', None))
      response = 'ack'

    elif cmd == 'get_view':
      response = self.get_view()

    elif cmd == 'scopeserver_status':
      response = 'ack'

    elif cmd == 'reset_server':
      response = 'ack'

    elif cmd == 'quit_server':
      response = 'ack'
      self.server_stop()

    elif cmd == 'reset_autoguider':
      response = 'ack'
      self.autoguider_q.put(('reset_autoguider', None))

    elif cmd == 'reboot_autoguider':
      response = 'ack'
      self.autoguider_q.put(('reboot_autoguider', None))

    elif cmd == 'shutdown_autoguider':
      response = 'ack'
      self.autoguider_q.put(('shutdown_autoguider', None))

    else:
      pass

    return response


  def init_cam(self):
    sys.stderr.write('\r\nConfiguring Camera...\r\n')

    self.sensor_dimx = 3.68
    self.sensor_dimy = 2.76
    self.npix_x = 3280
    self.npix_y = 2464
    self.arcsec_per_pixel = (3600.0/self.npix_x)*2*np.arctan(self.sensor_dimx/(2.0*240))*180.0/np.pi
    self.fov_x = self.npix_x*self.arcsec_per_pixel
    self.fov_y = self.npix_y*self.arcsec_per_pixel
    self.ra_counts_per_arcsec = 40000*576/(360*3600.)
    self.ra_arcsec_per_count = 1/self.ra_counts_per_arcsec
    self.dec_counts_per_arcsec = 40000*450/(360*3600.)
    self.dec_arcsec_per_count = 1/self.dec_counts_per_arcsec
    self.ra_counts_per_pixel = self.ra_counts_per_arcsec*self.arcsec_per_pixel
    self.dec_counts_per_pixel = self.dec_counts_per_arcsec*self.arcsec_per_pixel

    self.cam = picamera.PiCamera()
    self.cam.sensor_mode = 2
    self.cam.resolution = (self.npix_x, self.npix_y)
    self.cam.rotation = 180*(self.meridian_mode==1)

#    self.exposure = 1.0  # exposure time in seconds

    self.cam.framerate = 1/self.exposure
    self.cam.shutter_speed = int(self.exposure*1e6/1.0)
    self.cam.exposure_compensation = 24  # increase exposure by 4 stops
#    self.cam.iso = 800
    self.cam.iso = 0
    self.cam.analog_gain = 10.0
    self.cam.digital_gain = 1.0
    self.cam.meter_mode = 'average'
    self.cam.exposure_mode = 'night'
    self.cam.image_denoise = False
    time.sleep(5)
    self.cam.exposure_mode = 'off'
#    g = self.cam.awb_gains
    g = (1.1,2.0)
    self.cam.awb_mode = 'off'
    self.cam.awb_gains = g

    # Capture to numpy array:
    img_y = np.empty((2464, 3296), dtype=np.uint8)
    try:
      sys.stderr.write('Capturing test image to numpy array  \r\n')
      self.cam.capture(img_y, use_video_port=True, format='yuv')
    except IOError:
      pass
    img = img_y[:2464, :3280]
    sys.stderr.write('  Captured: %s\r\n' % (str(img.shape)))

    sys.stderr.write('Camera ready:\r\n')
    sys.stderr.write('  Exposure: %s\r\n' % (str(self.cam.exposure_speed)))
    sys.stderr.write('  ISO: %s (gains fixed)\r\n' % (str(self.cam.iso)))
    sys.stderr.write('  Digital gain: %s\r\n' % (str(float(self.cam.digital_gain))))
    sys.stderr.write('  Analog gain: %s\r\n' % (str(float(self.cam.analog_gain))))
    sys.stderr.write('  AWB gains: %s\r\n' % (str(self.cam.awb_gains)))

    self.img_count = 0
    self.guider_view = 'data:image/jpg;base64,'
    self.image = None
    self.image_c = None
    self.cdf = None
    self.guide_star_rect = None
    self.guide_star_rect_xy = None
    self.make_guider_view()


  def set_gamma_lut(self, gamma=1.0, blackpoint=0):

    bp = blackpoint
    invGamma = 1.0/gamma
    self.gamma_lut = np.zeros((256,),dtype='uint8')
    self.gamma_lut[bp:] = np.array([0.5+(((i-bp)/(255.0-bp))**invGamma)*255 for i in np.arange(bp,256)]).astype('uint8')

#    return cv2.LUT(image, gamma_lut)
    return



#  def nightshot(self):
  def nightshot_jpeg(self):

    # Capture to memory buffer:
    img_buf = io.BytesIO()
#    if self.verbose:
#      sys.stderr.write('  Capturing image to buffer  \r\n')
    t0 = time.time()
#    sys.stderr.write('\r\nCapturing image at t: %s\r\n' % (str(t0)))
    self.cam.rotation = 180*(self.meridian_mode==1)
    self.cam.capture(img_buf, use_video_port=True, format='jpeg', quality=75)
    img_buf.truncate()
    img_buf.seek(0)
    img_data = np.frombuffer(img_buf.getvalue(), dtype=np.uint8)
#    img_buf.seek(0)
#    img_buf.truncate()
#    img = cv2.imdecode(img_data, cv2.IMREAD_GRAYSCALE)
    img_c = cv2.imdecode(img_data, cv2.IMREAD_UNCHANGED)
    img_g = cv2.cvtColor(img_c, cv2.COLOR_BGR2GRAY)
    cv2.normalize(img_g, img_g, 0, 255, cv2.NORM_MINMAX)
#    img = np.flipud(img)
    self.image = img_g
    self.image_c = img_c
    self.prev_capture_time = self.capture_time
    self.capture_time = time.time_ns()
#    ofn = './images/image.jpg'
#    cv2.imwrite(ofn, img, (cv2.IMWRITE_JPEG_QUALITY, 75))
#    if self.verbose:
#      sys.stderr.write('      Captured: %s\r\n' % (str(self.image_c.shape)))

#    img = cv2.resize(img,(1600,1200))
#    img_buf = io.BytesIO(cv2.imencode(".jpg", img, (cv2.IMWRITE_JPEG_QUALITY, 75))[1])
#    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')

#    if self.status != 'active_analyzing':
#      self.make_guider_view()

    '''
    dt = time.time() - t0
    delay = self.autoguider_interval - dt
    sys.stderr.write('    delaying: %s\r\n' % (str(delay)))
    if delay > 0.0:
      time.sleep(delay)
    '''

    delay = self.autoguider_interval - self.exposure
#    sys.stderr.write('    delaying: %s\r\n' % (str(delay)))
    if delay > 0.0:
#      time.sleep(delay)
      pass

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


# nightshot in YUV Mode
#  def nightshot_y(self):
  def nightshot(self):

    # Capture to numpy array:
#    if self.verbose:
#      sys.stderr.write('  Capturing image to numpy array  \r\n')
    t0 = time.time()
#    sys.stderr.write('\r\nCapturing image at t: %s\r\n' % (str(t0)))
    self.cam.rotation = 180*(self.meridian_mode==1)
    img_y = np.empty((2464, 3296), dtype=np.uint8)
    try:
      self.cam.capture(img_y, use_video_port=True, format='yuv')
    except IOError:
      pass
    img_g = img_y[:2464, :3280]
    img_y = None
#    cv2.normalize(img_g, img_g, 0, 255, cv2.NORM_MINMAX)
#    img = np.flipud(img)
    self.image = img_g
    img_g = None
    # self.image_c = img_g
    self.prev_capture_time = self.capture_time
    self.capture_time = time.time_ns()
#    ofn = './images/image.jpg'
#    cv2.imwrite(ofn, img, (cv2.IMWRITE_JPEG_QUALITY, 75))
#    if self.verbose:
#      sys.stderr.write('      Captured: %s\r\n' % (str(self.image.shape)))

    delay = self.autoguider_interval - self.exposure
#    sys.stderr.write('    delaying: %s\r\n' % (str(delay)))
    if delay > 0.0:
#      time.sleep(delay)
      pass

    self.img_count += 1
    gc.collect()



  def nightshot_sim(self):

    t0 = time.time()
#    sys.stderr.write('\r\nReading sim image at t: %s\r\n' % (str(t0)))
    time.sleep(0.35*self.exposure)
    my_path = os.path.split(os.path.realpath(__file__))[0]
    img_dir = os.path.join(my_path,'./images_2/')
    img_fns = sorted(glob.glob(img_dir + '*'))
#    img_dir = './images_sim/'
#    img_fns = sorted(glob.glob(img_dir + '*'))

    i = self.img_count % len(img_fns)
    img_fn = img_fns[i]
 
    if self.verbose:
      sys.stderr.write('Reading sim image: %s\r\n' % (img_fn))


#    img_g = cv2.imread(img_fn, cv2.IMREAD_GRAYSCALE)
    img_c = cv2.imread(img_fn, cv2.IMREAD_UNCHANGED)
    if self.meridian_mode == -1:
      img_c = cv2.rotate(img_c, cv2.ROTATE_180)
    img_g = cv2.cvtColor(img_c, cv2.COLOR_BGR2GRAY)
    cv2.normalize(img_g, img_g, 0, 255, cv2.NORM_MINMAX)
    self.image = img_g
    self.image_c = img_c
    self.prev_capture_time = self.capture_time
    self.capture_time = time.time_ns()

    if self.verbose:
      sys.stderr.write('Sim Image: %s\r\n' % (str(self.image_c.shape)))

#    img = cv2.imread(img_fn)
#    img = np.flipud(img)
#    self.image = img

    '''
#    pil_img = Image.fromarray(img).resize((1600,1200)).convert('L')
    pil_img = Image.fromarray(img).resize((1600,1200))
    font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 48)
    draw = ImageDraw.Draw(pil_img)
    draw.text((100,100), self.status, (0xd0,0,0), font=font)
    del draw
    img_buf = io.BytesIO()
    pil_img.save(img_buf,"JPEG",quality=75)
    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')
    '''

    '''
    img = cv2.resize(img,(1600,1200))
    img =cv2.putText(img=np.copy(img), text=str(self.img_count), org=(100,150),fontFace=1, fontScale=4, color=(0,0,0xd0), thickness=4)
    img_buf = io.BytesIO(cv2.imencode(".jpg", img, (cv2.IMWRITE_JPEG_QUALITY, 75))[1])
    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')
    '''

#    self.make_guider_view()

    '''
    dt = time.time() - t0
    delay = self.autoguider_interval - dt
    sys.stderr.write('    delaying: %s\r\n' % (str(delay)))
    if delay > 0:
      time.sleep(delay)
    '''

    delay = self.autoguider_interval - self.exposure
#    sys.stderr.write('    delaying: %s\r\n' % (str(delay)))
    if delay > 0.0:
#      time.sleep(delay)
      pass

    self.img_count += 1


  def make_guider_view(self):
    view_w = 1600
    view_h = 1200
#    if type(self.image_c) == type(None):
    if type(self.image) == type(None):
#      img = np.zeros((self.npix_y,self.npix_x,3),dtype=np.uint8)
      self.image = np.zeros((self.npix_y,self.npix_x,1),dtype=np.uint8)

    if self.mag == 1:
      mag_w = self.npix_x
      mag_h = self.npix_y
      x = 0
      y = 0
      img = cv2.resize(self.image,(view_w, view_h))
    elif self.mag == 2:
      mag_w = 1600
      mag_h = 1200
      x = int((self.npix_x-mag_w)/2)
      y = int((self.npix_y-mag_h)/2)
      img = self.image[y:(y+mag_h), x:(x+mag_w)]
    elif self.mag == 3:
      mag_w = 1067
      mag_h = 800
      x = int((self.npix_x-mag_w)/2)
      y = int((self.npix_y-mag_h)/2)
      img = self.image[y:(y+mag_h), x:(x+mag_w)]
      img = cv2.resize(img,(view_w, view_h))
    elif self.mag == 4:
      mag_w = 800
      mag_h = 600
      x = int((self.npix_x-mag_w)/2)
      y = int((self.npix_y-mag_h)/2)
      img = self.image[y:(y+mag_h), x:(x+mag_w)]
      img = cv2.resize(img,(view_w, view_h))

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    '''
    if type(self.cdf) == type(None):
      hist,bins = np.histogram(img.flatten(),256,[0,256])
      cdf = hist.cumsum()
      cdf_m = np.ma.masked_equal(cdf,0)
      cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
      self.cdf = np.ma.filled(cdf_m,0).astype('uint8')
    img = self.cdf[img]
    '''
#    img = img*2
#    img = cv2.equalizeHist(img)
#    R, G, B = cv2.split(img)
#    Req = cv2.equalizeHist(R)
#    Geq = cv2.equalizeHist(G)
#    Beq = cv2.equalizeHist(B)
#    img = cv2.merge((Req, Geq, Beq))
#    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    self.view_scale = float(view_w)/mag_w
    x_offset = -x
    y_offset = -y

    img = cv2.LUT(img,self.gamma_lut)

    pil_img = Image.fromarray(img)
#    pil_img = Image.fromarray(img).resize((1600,1200))
    font = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 48)
    draw = ImageDraw.Draw(pil_img)
    text_label = '???'
    if self.status == 'idle':
      text_label = 'Idle'
    elif self.status == 'imaging':
      text_label = 'Imaging'
    elif self.status == 'finding_guide_star':
      text_label = 'Finding Guide Star'
    elif self.status == 'found_guide_star':
      text_label = 'Guide Star'
    elif self.status == 'no_guide_star_found':
      text_label = 'No Guide Star Found'
    elif self.status == 'active_guiding':
      text_label = 'Guiding'
    elif self.status == 'active_analyzing':
      text_label = 'Analyzing'
    elif self.status == 'init_cam':
      text_label = 'Init Camera'

    #Status labels
    text_label = '%s %d\n\u03B3=%g\nbp=%d\nmag=%d\nexp=%g\ndt=%g' % (text_label, self.img_count, self.gamma, self.blackpoint, self.mag, self.exposure, self.autoguider_interval)
    draw.text((50,50), text_label, (0xd0,0,0), font=font)

    #NSEW labels
    n_label = 'N'
    s_label = 'S'
    e_label = 'E'
    w_label = 'W'
    draw.text((785,4), n_label, (0xd0,0,0), font=font)
    draw.text((785,1140), s_label, (0xd0,0,0), font=font)
    draw.text((10,575), e_label, (0xd0,0,0), font=font)
    draw.text((1542,575), w_label, (0xd0,0,0), font=font)

    '''
    if self.meridian_mode == 1:
      draw.text((785,4), n_label, (0xd0,0,0), font=font)
      draw.text((785,1140), s_label, (0xd0,0,0), font=font)
      draw.text((10,575), e_label, (0xd0,0,0), font=font)
      draw.text((1542,575), w_label, (0xd0,0,0), font=font)
    else:
      draw.text((785,1140), n_label, (0xd0,0,0), font=font)
      draw.text((785,4), s_label, (0xd0,0,0), font=font)
      draw.text((1542,575), e_label, (0xd0,0,0), font=font)
      draw.text((10,575), w_label, (0xd0,0,0), font=font)
    '''

    #Edges of image
    draw.line(((0,0),(1599,0)),fill=(0xd0,0,0),width=1)
    draw.line(((0,1199),(1599,1199)),fill=(0xd0,0,0),width=1)
    draw.line(((0,0),(0,1199)),fill=(0xd0,0,0),width=1)
    draw.line(((1599,0),(1599,1199)),fill=(0xd0,0,0),width=1)

    arcmin = int(self.view_scale*(60.0/self.arcsec_per_pixel))
    qarcmin = int(self.view_scale*(15.0/self.arcsec_per_pixel))

    #Draw Cross hairs
    draw.line(((800,0),(800,1199)),fill=(0xd0,0,0),width=1)
    draw.line((((800+arcmin),0),((800+arcmin),1199)),fill=(0xd0,0,0),width=1)
    draw.line((((800-arcmin),0),((800-arcmin),1199)),fill=(0xd0,0,0),width=1)

    draw.line(((0,600),(1599,600)),fill=(0xd0,0,0),width=1)
    draw.line(((0,(600+arcmin)),(1599,(600+arcmin))),fill=(0xd0,0,0),width=1)
    draw.line(((0,(600-arcmin)),(1599,(600-arcmin))),fill=(0xd0,0,0),width=1)

    #Draw Tick Marks
    #  Major Ticks X
    for x in range((-20*arcmin), (20*arcmin+1), (10*arcmin)):
      draw.line(((800+x,(600-2*arcmin)),(800+x,(600+2*arcmin))),fill=(0xd0,0,0),width=1)

    #  Secondary Ticks X
    for x in range((-25*arcmin), (25*arcmin+1), (5*arcmin)):
      draw.line(((800+x,(600-1.5*arcmin)),(800+x,(600+1.5*arcmin))),fill=(0xd0,0,0),width=1)

    #  Minor Ticks X
    for x in range((-26*arcmin), (26*arcmin+1), (1*arcmin)):
      draw.line(((800+x,(600-arcmin)),(800+x,(600+arcmin))),fill=(0xd0,0,0),width=1)
      #  Micro Ticks X
      if self.mag > 1:
        for x_mic in range(x, x+arcmin-4, qarcmin):
          draw.line(((800+x_mic,(600-qarcmin)),(800+x_mic,(600+qarcmin))),fill=(0xd0,0,0),width=1)


    #  Major Ticks Y
    for y in range((-20*arcmin), (20*arcmin+1), (10*arcmin)):
      draw.line((((800-2*arcmin),600+y),((800+2*arcmin),600+y)),fill=(0xd0,0,0),width=1)

    #  Secondary Ticks Y
    for y in range((-20*arcmin), (20*arcmin+1), (5*arcmin)):
      draw.line((((800-1.5*arcmin),600+y),((800+1.5*arcmin),600+y)),fill=(0xd0,0,0),width=1)

    #  Minor Ticks Y
    for y in range((-20*arcmin), (20*arcmin+1), (arcmin)):
      draw.line((((800-arcmin),600+y),((800+arcmin),600+y)),fill=(0xd0,0,0),width=1)
      #  Micro Ticks Y
      if self.mag > 1:
        for y_mic in range(y, y+arcmin-4, qarcmin):
          draw.line((((800-qarcmin),600+y_mic),((800+qarcmin),600+y_mic)),fill=(0xd0,0,0),width=1)
    
    #Draw Pick Circles
    if self.img_pick:
      r = 30
      position = self.pick_pnt_1[:-1]
      xy = np.array([position-[r,r], position+[r,r]])
      xy = tuple(map(tuple, xy))
      draw.ellipse(xy, fill=None,outline=(0xd0,0,0), width=1)
      draw.text(position-[14,27], '1', (0xd0,0,0), font=font)
      if type(self.pick_pnt_2) != type(None):
        position = self.pick_pnt_2[:-1]
        xy = np.array([position-[r,r], position+[r,r]])
        xy = tuple(map(tuple, xy))
        draw.ellipse(xy, fill=None,outline=(0xd0,0,0), width=1)
        draw.text(position-[14,27], '2', (0xd0,0,0), font=font)

    #Draw Inscribed Circle
    r = 599.0
    position = np.array((799.5,599.5))
    xy = np.array([position-[r,r],position+[r,r]])
    xy = tuple(map(tuple, xy))
    draw.ellipse(xy,fill=None,outline=(0xd0,0,0),width=1)

    # Box guide star and circle minor stars
    if (self.status == 'active_analyzing'):
        xy = ( (x_offset, y_offset) + self.guide_star_rect_xy + np.array((self.driftRA,self.driftDEC)) )*self.view_scale
        xy = tuple(map(tuple, xy))
        draw.rectangle(xy,fill=None,outline=(0xd0,0,0),width=3)
    elif (self.status == 'found_guide_star') or (self.status == 'active_guiding'):
      if type(self.guide_star_rect_xy) != type(None):
        xy = ((x_offset, y_offset) + self.guide_star_rect_xy)*self.view_scale
        xy = tuple(map(tuple, xy))
        draw.rectangle(xy,fill=None,outline=(0xd0,0,0),width=3)
        # Draw X in Box
        draw.line((xy[0],xy[1]),fill=(0xd0,0,0),width=1)
        draw.line(((xy[0][0],xy[1][1]),(xy[1][0],xy[0][1])),fill=(0xd0,0,0),width=1)
        if self.status == 'found_guide_star':
          for position in self.positions:
            r = 16.0
            xy = np.array([position-[r,r],position+[r,r]])
            xy = ((x_offset, y_offset) + xy)*self.view_scale
            xy = tuple(map(tuple, xy))
            draw.ellipse(xy,fill=None,outline=(0xd0,0,0),width=3)
    del draw
    img_buf = io.BytesIO()
    pil_img.save(img_buf,"JPEG",quality=75)
    self.guider_view = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')


  def plot_drift_analysis(self):
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

    t = self.drift_ana[1:,0]
    ra = self.drift_ana[1:,1]
    dec = self.drift_ana[1:,2]
    pex_idx = self.drift_ana[1:,3]

    if len(t) < 300:
      ax.scatter(t, ra, marker='o', s=25, edgecolors='#d00000', facecolors='none')
      ax.scatter(t, dec, marker='^', s=25, edgecolors='#d00000', facecolors='none')
    else:
      ax.scatter(t[-300:], ra[-300:], marker='o', s=25, edgecolors='#d00000', facecolors='none')
      ax.scatter(t[-300:], dec[-300:], marker='^', s=25, edgecolors='#d00000', facecolors='none')

    if len(t) > 1:
      t_lin = np.array([t.min(),t.max()])
      (m_ra,b,r,p,stderr) = sps.linregress(t, ra)
      mm_ra = to_precision(m_ra,2)
      ra_lin = (m_ra*t_lin) + b
      (m_dec,b,r,p,stderr) = sps.linregress(t, dec)
      c_dec = (self.fov_x/60.)*m_dec/0.42
      mm_dec = to_precision(m_dec,2)
      cc_dec = to_precision(c_dec,2)
      dec_lin = (m_dec*t_lin) + b
      if len(t) > 300:
        t_lin = np.array([t[-300],t[-1]])
        ra_lin = (m_ra*t_lin) + b
        dec_lin = (m_dec*t_lin) + b
      ax.plot(t_lin, ra_lin, color='#d00000')
      ax.plot(t_lin, dec_lin, color='#d00000', linestyle='--')

      ax.set_xlabel(r'time (s)',labelpad=1)
      ax.set_ylabel(r'displacement (arcsec)',labelpad=1)

      if (c_dec > 0):  # Drifting South
        if self.meridian_mode == 1:
          corr_orient = '(W,S)'
        else:
          corr_orient = '(W,N)'
      else:  # Drifting North
        if self.meridian_mode == 1:
          corr_orient = '(E,N)'
        else:
          corr_orient = '(E,S)'

      ax.text(0.05, 0.95, "m_ra = %s\nm_dec = %s\ncorr = %s' %s" % (mm_ra, mm_dec, cc_dec, corr_orient),
      verticalalignment='top', horizontalalignment='left',
      transform = ax.transAxes, fontsize=10, color='#d00000')


    img_buf = io.BytesIO()
    plt.savefig(img_buf, format='png')
    plt.close()
    self.guider_view = 'data:image/png;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')


  def find_guide_star(self):
    self.find_guide_star_v2()


  def find_guide_star_v2(self):  

    self.make_guider_view()

    if self.verbose:
      sys.stderr.write('Finding Guide Star...\r\n')

    self.ref_image = self.image
    img = self.ref_image

#    w = 128
#    h = 128
    w = 256
    h = 256

#    limg = np.arcsinh(img).astype('float32')
    limg = img.astype('float32')
    limg = limg / limg.max()
#    low = np.percentile(limg, 0.25)
#    high = np.percentile(limg, 99.5)
#    opt_img  = skie.exposure.rescale_intensity(limg, in_range=(low,high))

#    lm = morph.is_local_maximum(limg)
#    x1, y1 = np.where(lm.T == True)
    y1, x1 = morph.local_maxima(limg,connectivity=1,indices=True)
    v = limg[(y1,x1)]
#    lim = 0.78
    lim = 0.25
#    lim = 0.15
    sources = np.array((x1[v > lim], y1[v > lim])).transpose()
    sources = sources[(limg[sources[:,1],sources[:,0]]).argsort()[::-1]]
#    print(sources)

    if len(sources) == 0:
      self.status = 'no_guide_star_found'
      self.make_guider_view()
      if self.verbose:
        sys.stderr.write('\r\nNo Guide Star Found\r\n')
      return

    self.guide_star_pos = sources[0]
    self.guide_star_rect = np.array([self.guide_star_pos-[w/2.,h/2.],w,h])
    self.guide_star_rect_xy = np.array( [self.guide_star_pos-[w/2.,h/2.],self.guide_star_pos+[w/2.,h/2.]] )
    self.positions = sources

    swim_dir = os.path.join(self.dir_path,'swim_tmp')
    img_ref_fn = os.path.join(swim_dir,'guide_ref.jpg')
    self.image_crop_rect(img, self.guide_star_rect, img_ref_fn)

    self.status = 'found_guide_star'
    self.make_guider_view()
    if self.verbose:
      sys.stderr.write('Found Guide Star\r\n')



  def find_guide_star_v1(self):

    self.make_guider_view()

    if self.verbose:
      sys.stderr.write('Finding Guide Star...\r\n')

    self.ref_image = self.image
    img = self.ref_image

#    w = 128
#    h = 128
    w = 256
    h = 256

    mean, median, std = sigma_clipped_stats(img, sigma=3.0)
    bkg_sigma = mad_std(img)
#    print((mean, median, std))
#    print(bkg_sigma)

    daofind = DAOStarFinder(fwhm=9.0, sharphi=0.75, threshold=30.0*bkg_sigma, exclude_border=True)
#    daofind = DAOStarFinder(fwhm=9.0, sharphi=0.65, threshold=20.0*bkg_sigma, exclude_border=True)

    try:
      sources = daofind(img - median)
      for col in sources.colnames:
        sources[col].info.format = '%.8g'  # for consistent table output

      sources.sort(['peak'])
      sources.reverse()
    except:
      self.status = 'no_guide_star_found'
      self.make_guider_view()
      if self.verbose:
        sys.stderr.write('\r\nNo Guide Star Found\r\n')
      return

#    print('')
#    print('All sources:')
#    print(sources)

#    print('')
#    print('Guide Source:')
#    print(sources[0])

    if len(sources) == 0:
      self.status = 'no_guide_star_found'
      self.make_guider_view()
      if self.verbose:
        sys.stderr.write('\r\nNo Guide Star Found\r\n')
      return

    self.guide_star_pos = np.array([sources['xcentroid'][0],sources['ycentroid'][0]])
    self.guide_star_rect = np.array([self.guide_star_pos-[w/2.,h/2.],w,h])
    self.guide_star_rect_xy = np.array( [self.guide_star_pos-[w/2.,h/2.],self.guide_star_pos+[w/2.,h/2.]] )
    self.positions = np.transpose((sources['xcentroid'], sources['ycentroid']))

    swim_dir = os.path.join(self.dir_path,'swim_tmp')
    img_ref_fn = os.path.join(swim_dir,'guide_ref.jpg')
    self.image_crop_rect(img, self.guide_star_rect, img_ref_fn)

    '''
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
    '''

    self.status = 'found_guide_star'
    self.make_guider_view()
    if self.verbose:
      sys.stderr.write('Found Guide Star\r\n')


  def image_crop_rect(self, img, rect, img_out_fn):
    x = int(rect[0][0])
    y = int(rect[0][1])
    w = int(rect[1])
    h = int(rect[2])
    img_cropped = img[y:(y+h), x:(x+w)]
    cv2.imwrite(img_out_fn, img_cropped)


  def guide_star_drift_analysis(self):

    test_img_dir = './images_3/'
    test_img_fns = sorted(glob.glob(test_img_dir + '*'))

    swim_dir = os.path.join(self.dir_path,'swim_tmp')
    img_ref_fn = os.path.join(swim_dir,'guide_ref.jpg')
    img_update_fn = os.path.join(swim_dir,'guide_update.jpg')
    img_keep_fn = os.path.join(swim_dir,'keep.JPG')

    guide_star_dat = np.empty((0,5))
    guide_star_dat = np.append(guide_star_dat,[0,0,0,0,0]).reshape(-1,5)

    dx = self.driftRA
    dy = self.driftDEC
#    w = 128
#    h = 128
    w = 256
    h = 256

    guide_update_rect = np.array([self.guide_star_pos+[dx,dy]-[w/2.,h/2.],w,h])
    self.image_crop_rect(self.image, guide_update_rect, img_update_fn)

    swim_cmd_template = '/home/pi/bin/swim %d -i 2 -w -0.71 %s %s'
    swim_cmd = swim_cmd_template % (w, img_ref_fn, img_update_fn)

#    swim_cmd_template = '/home/pi/bin/swim %d -i 2 -w -0.71 -keep %s %s %s'
#    swim_cmd = swim_cmd_template % (w, img_keep_fn, img_ref_fn, img_update_fn)

#    sys.stderr.write('%s\r\n' % (swim_cmd))
    swim_log = sp.run([swim_cmd],cwd=swim_dir,shell=True,stdout=sp.PIPE,stderr=sp.PIPE)
    swim_stdout = swim_log.stdout.decode('utf-8')
    swim_stderr = swim_log.stdout.decode('utf-8')
    toks = swim_stdout.split()
#    print('swim output: \n  ' + swim_stdout)
    snr = float(toks[0][0:-1])
    dxi = float(toks[8][1:])
    dyi = float(toks[9])
#    self.driftRA += int(dxi*self.ra_counts_per_pixel)
#    self.driftDEC += int(dyi*self.dec_counts_per_pixel)
    self.driftRA += dxi
    self.driftDEC += dyi
    driftRA_arcsec = self.driftRA*self.arcsec_per_pixel
    driftDEC_arcsec = self.driftDEC*self.arcsec_per_pixel
    if len(self.drift_ana) == 0:
      self.drift_ana = np.append(self.drift_ana, np.array([[0.0,driftRA_arcsec,driftDEC_arcsec,self.pec_idx]]), axis=0)
    elif len(self.drift_ana) == 1:
      self.drift_ana = np.append(self.drift_ana, np.array([[0.0,driftRA_arcsec,driftDEC_arcsec,self.pec_idx]]), axis=0)
    else:
      dt = (self.capture_time - self.prev_capture_time)*1e-9
      t = dt + self.drift_ana[-1][0]
      self.drift_ana = np.append(self.drift_ana, np.array([[t,driftRA_arcsec,driftDEC_arcsec,self.pec_idx]]), axis=0)

    sys.stderr.write('swim match:  SNR: %g  driftRA: %.4g  driftDEC: %.4g\r\n' % (snr, self.driftRA, self.driftDEC))
    if self.verbose:
      sys.stderr.write('swim match:  SNR: %g  driftRA: %.4g  driftDEC: %.4g\r\n' % (snr, self.driftRA, self.driftDEC))

    self.plot_drift_analysis()


  def guide_star_drift(self):

    test_img_dir = './images_3/'
    test_img_fns = sorted(glob.glob(test_img_dir + '*'))

    swim_dir = os.path.join(self.dir_path,'swim_tmp')
    img_ref_fn = os.path.join(swim_dir,'guide_ref.jpg')
    img_update_fn = os.path.join(swim_dir,'guide_update.jpg')
    img_keep_fn = os.path.join(swim_dir,'keep.JPG')

    guide_star_dat = np.empty((0,5))
    guide_star_dat = np.append(guide_star_dat,[0,0,0,0,0]).reshape(-1,5)

    dx = 0
    dy = 0
#    w = 128
#    h = 128
    w = 256
    h = 256

    guide_update_rect = np.array([self.guide_star_pos+[dx,dy]-[w/2.,h/2.],w,h])
    self.image_crop_rect(self.image, guide_update_rect, img_update_fn)

    swim_cmd_template = '/home/pi/bin/swim %d -i 2 -w -0.71 -keep %s %s %s'
    swim_cmd = swim_cmd_template % (w, img_keep_fn, img_ref_fn, img_update_fn)
#    sys.stderr.write('%s\r\n' % (swim_cmd))
    swim_log = sp.run([swim_cmd],cwd=swim_dir,shell=True,stdout=sp.PIPE,stderr=sp.PIPE)
    swim_stdout = swim_log.stdout.decode('utf-8')
    swim_stderr = swim_log.stdout.decode('utf-8')
    toks = swim_stdout.split()
#    print('swim output: \n  ' + swim_stdout)
    snr = float(toks[0][0:-1])
    dxi = float(toks[8][1:])
    dyi = float(toks[9])
    dx += dxi
    dy += dyi
    self.dRA = int(0.5*dxi*self.ra_counts_per_pixel)
    self.dDEC = int(0.5*dyi*self.dec_counts_per_pixel)
    self.correction_log.write('%d %d\n' % (self.dRA, self.dDEC))

    sys.stderr.write('swim match:  SNR: %g  dRA: %d  dDEC: %d\r\n' % (snr, self.dRA, self.dDEC))
    if self.verbose:
      sys.stderr.write('swim match:  SNR: %g  dRA: %d  dDEC: %d\r\n' % (snr, self.dRA, self.dDEC))

#    guide_star_dat = np.append(guide_star_dat,[t,dx,dy,dxi,dyi]).reshape(-1,5)
    if len(self.drift_corr) == 0:
      self.drift_corr = np.append(self.drift_corr, np.array([[0.0,self.dRA,self.dDEC]]), axis=0)
    elif len(self.drift_corr) == 1:
      self.drift_corr = np.append(self.drift_corr, np.array([[0.0,self.dRA,self.dDEC]]), axis=0)
    else:
      dt = (self.capture_time - self.prev_capture_time)*1e-9
      t = dt + self.drift_corr[-1][0]
      self.drift_corr = np.append(self.drift_corr, np.array([[t,self.dRA,self.dDEC]]), axis=0)

    if self.gcorr_plot:
      self.plot_guide_star_corrections()
    else:
      self.make_guider_view()


  def plot_guide_star_corrections(self):
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

    ax2 = ax.twinx() 
    ax2.spines['bottom'].set_color('#d00000')
    ax2.spines['top'].set_color('#d00000') 
    ax2.spines['right'].set_color('#d00000')
    ax2.spines['left'].set_color('#d00000')
    ax2.tick_params(axis='x', colors='#d00000', which='both')
    ax2.tick_params(axis='y', colors='#d00000', which='both')
    ax2.yaxis.label.set_color('#d00000')
    ax2.xaxis.label.set_color('#d00000')

    t = self.drift_corr[:,0]
    ra = self.drift_corr[:,1]
    dec = self.drift_corr[:,2]

    ax.scatter(t, ra, marker='o', s=25, edgecolors='#d00000', facecolors='none')
    ax.scatter(t, dec, marker='^', s=25, edgecolors='#d00000', facecolors='none')

    if len(t) > 1:
      t_lin = np.array([t.min(),t.max()])
      (m_ra,b,r,p,stderr) = sps.linregress(t, ra)
      mm_ra = to_precision(m_ra,2)
      ra_lin = (m_ra*t_lin) + b
      (m_dec,b,r,p,stderr) = sps.linregress(t, dec)
      mm_dec = to_precision(m_dec,2)
      dec_lin = (m_dec*t_lin) + b
      ax.plot(t_lin, ra_lin, color='#d00000')
      ax.plot(t_lin, dec_lin, color='#d00000', linestyle='--')

      ax.set_xlabel(r'time (s)',labelpad=1)
      ax.set_ylabel(r'guide correction (counts)',labelpad=1)

      ax2.set_ylabel(r'guide correction (arcsecs)',labelpad=1)

      ax.set_ylim([np.min((ra.min(),dec.min())), np.max((ra.max(),dec.max()))])
      ax2.set_ylim([self.ra_arcsec_per_count*np.min((ra.min(),dec.min())), self.ra_arcsec_per_count*np.max((ra.max(),dec.max()))])

      ax.text(0.05, 0.95, 'm_ra = %s\nm_dec = %s' % (mm_ra, mm_dec),
      verticalalignment='top', horizontalalignment='left',
      transform = ax.transAxes, fontsize=10, color='#d00000')


    img_buf = io.BytesIO()
    plt.savefig(img_buf, format='png')
    plt.close()
    self.guider_view = 'data:image/png;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')


  def scopeserver_guide_star_correction(self, reset_target = False):
    HOST = self.scopeserver_host
    PORT = self.scopeserver_port
    cmd = '{guide_star_correction %d %d %s}' % (self.dRA, self.dDEC, str(reset_target))

    if self.verbose:
      sys.stderr.write('\r\nAuto Guider Sending:  %s\r\n' % (cmd))

    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      try:
        sock.settimeout(0.1)
        sock.connect((HOST, PORT))
        sock.settimeout(None)
        sock.sendall(bytes(cmd, 'utf-8'))
    
        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8')
      except:
        response = ''
      finally:
        sock.close()

    if self.verbose:
      sys.stderr.write('  AutoGuider Correction Sent:  %s\r\n' % (cmd))
      sys.stderr.write('  AutoGuider Received:  %s\r\n' % (response))
    if response == 'ack':
      self.scopeserver_connected = True
    else:
      self.scopeserver_connected = False
      sys.stderr.write('Guide Star Correction Failed: ScopeServer Not Connected\r\n')
      sys.stderr.write('Attempting to Reconnect...\r\n')
      self.autoguider_q.put(('ss_connect', None))


  def get_scopeserver_status(self):
    HOST = self.scopeserver_host
    PORT = self.scopeserver_port
    cmd = '{get_small_status}'

#    sys.stderr.write('\r\nAuto Guider Sending:    %s\r\n' % (cmd))
    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      try:
        sock.settimeout(0.1)
        sock.connect((HOST, PORT))
        sock.settimeout(None)
        sock.sendall(bytes(cmd, 'utf-8'))
    
        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8')
      except:
        response = ''
      finally:
        sock.close()

#    sys.stderr.write('Auto Guider Sent:       %s\r\n' % (cmd))
#    sys.stderr.write('Auto Guider Received:   %s\r\n' % (response))
    if response:
      status = eval(response)
      self.pos_ra = int(status['ra_pos'])
      self.pos_dec = int(status['dec_pos'])
      self.pec_idx = int(status['pec_idx'])
      if int(status['meridian_mode']) != self.meridian_mode:
        meridian_mode = int(status['meridian_mode'])
        self.autoguider_q.put(('update_meridian_mode', meridian_mode))
#      sys.stderr.write('2) setting meridian_mode: %d\r\n' % (meridian_mode))
      self.scopeserver_connected = True
    else:
      self.scopeserver_connected = False
      sys.stderr.write('Get ScopeServer Status Failed: ScopeServer Not Connected\r\n')


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
        try:
          sock.settimeout(0.1)
          sock.connect((HOST, PORT))
          sock.settimeout(None)
          t0 = time.time_ns()
          sock.sendall(bytes(cmd, 'utf-8'))

          # Receive response from the server and finish
          response = str(sock.recv(1024), 'utf-8').split()
          t2 = time.time_ns()
        except:
          response = []
        finally:
          sock.close()

      if len(response):
        t_ss = int(response[1])
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

  sim_img_mode=False
  if (len(sys.argv)==2):
    sim_img_mode=True

  pid = os.getpid()
  try:
    pidfile = open('/var/run/picam_autoguider.pid','w')
    pidfile.write('%d\n' % (pid))
    pidfile.close()
  except:
    pass

  server_address = get_ip()
  port = 54040

  if sim_img_mode:
    sys.stderr.write('\r\nStarting PiCAM AutoGuider at %s:%d in Simulated Image Mode\r\n' % (server_address, port))
  else:
    sys.stderr.write('\r\nStarting PiCAM AutoGuider at %s:%d\r\n' % (server_address, port))

  ag = autoguider(server_address, port, sim_img_mode=sim_img_mode)
#  ag = autoguider(server_address, port, sim_img_mode=True)
  ag.server_start()

