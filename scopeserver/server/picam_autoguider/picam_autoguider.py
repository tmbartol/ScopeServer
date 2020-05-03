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
import os
import queue
import subprocess
import picamera
from fractions import Fraction
import io

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


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
          cmd = data.decode('ascii').strip()
          if cmd:
            if cmd[0] == '{':
              response = ag.process_autoguider_cmd(cmd)
            if response:
              # Send response to client
#              sys.stderr.write('Server responding:  %s\r\n' % (response.encode('ascii')))
              self.request.sendall(response.encode('ascii'))
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


  # Start server thread
  def server_start(self):
    sys.stderr.write('\r\nConnecting to ScopeServer at %s:%d ...\r\n' %(self.scopeserver_host,self.scopeserver_port))
    self.scopeserver_connected = self.scopeserver_connect()
#    self.scopeserver_connected = False

    self.input_start()
    self.autoguider_control_start()
    self.server_running = True

    self.autoguider_q.put(('sync_time', None))

    self.threaded_server = ThreadedTCPServer((self.autoguider_host, self.autoguider_port), ThreadedTCPRequestHandler)

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    self.server_thread = threading.Thread(target=self.threaded_server.serve_forever)
    # Exit the server thread when the main thread terminates
    self.server_thread.daemon = True
    self.server_thread.start()
    sys.stderr.write('\r\nWaiting for a command, type q to quit...\r\n')
    sys.stderr.write('  h -> print this help message\r\n')
    sys.stderr.write('  c -> capture one image\r\n')
    sys.stderr.write('  g -> begin capture continuous\r\n')
    sys.stderr.write('  s -> stop capture continuous\r\n')
    sys.stderr.write('  t -> synchronize time with ScopeServer\r\n')
    sys.stderr.write('  q -> quit and shutdown Autoguider\r\n\r\n')


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

      elif cmd == 'autoguider_continue':
        pass

      if cmd == 'capture_one':
        self.nightshot()
        if (self.capture_continuous):
          self.autoguider_q.put(('capture_one', None))

      if cmd == 'sync_time':
        self.sync_time()

      if not cmd == 'autoguider_continue':
        self.autoguider_q.task_done()


  def scopeserver_connect(self):
    HOST = self.scopeserver_host
    PORT = self.scopeserver_port
    cmd = '{autoguider_connect}'

    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      sock.connect((HOST, PORT))
      sock.sendall(bytes(cmd, "utf-8"))
    
      # Receive response from the server and finish
      response = str(sock.recv(1024), "utf-8")

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

      # Handle Arrow keys
      # N, North, up arrow
      if k=='\x1b[A':
        if self.dec_axis_running:
          self.autoguider_q.put(('dec_slew_stop', None))
        else:
          self.autoguider_q.put(('dec_slew_start', -1))
#        sys.stderr.write('  moved N to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # c, capture one image
      elif k=='c':
        if (self.capture_continuous == False):
          self.autoguider_q.put(('capture_one', None))

      # g, begin capture continuously
      elif k=='g':
        if (self.capture_continuous == False):
          self.capture_continuous = True
          self.autoguider_q.put(('capture_one', None))

      # s, stop capture continuously
      elif k=='s':
        self.capture_continuous = False

      # Report Position
      elif k=='p':
        self.autoguider_q.put(('update_pos', True))
#        sys.stderr.write('  Current Pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Synchronize time with ScopeServer
      elif k=='t':
        self.autoguider_q.put(('sync_time', None))

      # Shutdown the server
      elif k=='q':
        self.input_running = False

      # Print help message
      elif k=='h':
        sys.stderr.write('\r\n>>> ' + str(k) + '\r\n')
        sys.stderr.write('   h -> print this help message\r\n')
        sys.stderr.write('   c -> capture one image\r\n')
        sys.stderr.write('   g -> begin capture continuous\r\n')
        sys.stderr.write('   s -> stop capture continuous\r\n')
        sys.stderr.write('   t -> synchronize time with ScopeServer\r\n')
        sys.stderr.write('   q -> quit and shutdown Autoguider\r\n')
      else:
        sys.stderr.write('>>> ' + str(k) + ' \r\n')
        sys.stderr.write('Unknown key! Please type q to quit.\r\n')
        sys.stderr.write('   h -> print this help message\r\n')
        sys.stderr.write('   c -> capture one image\r\n')
        sys.stderr.write('   g -> begin capture continuous\r\n')
        sys.stderr.write('   s -> stop capture continuous\r\n')
        sys.stderr.write('   t -> synchronize time with ScopeServer\r\n')
        sys.stderr.write('   q -> quit and shutdown Autoguider\r\n')
    self.server_stop()


  # Process BlueShift Telescope Auto_Guider Protocol Commands
  def process_autoguider_cmd(self,cmd):

    response = None

    cmd = cmd[1:-1]

    if cmd.split()[0] == 'get_autoguider_time':
      response = 'autoguider_time %s' % (str(time.time_ns()))
#      self.autoguider_q.put(('jog_pos', (jog_ra, jog_dec)))

    elif cmd == 'get_status':
      response = str(self.get_status())

    elif cmd == 'reset_server':
      pass

    elif cmd == 'shutdown_server':
      self.server_stop()

    elif cmd == 'reboot_system':
      pass

    elif cmd == 'shutdown_system':
      pass

    else:
      pass

    return response


  def init_cam(self):
    sys.stderr.write('\r\n')
    sys.stderr.write('Configuring Camera...\r\n')

    self.cam = picamera.PiCamera()
    self.cam.sensor_mode = 2
    self.cam.resolution = (3280,2464)
#    self.cam.resolution = (3296,2464)
    self.cam.rotation = 180
    #self.cam.zoom = (0.25,0.25,0.75,0.75)

    self.exposure = 1.0

    self.cam.framerate = 1/self.exposure
    self.cam.shutter_speed = int(self.exposure*1e6/1.0)
    self.cam.iso = 800
    self.cam.meter_mode = 'average'
    self.cam.exposure_mode = 'night'
    self.cam.image_denoise = False
    time.sleep(5)
    self.cam.exposure_mode = 'off'

    self.cam.capture('./init_image.jpg', use_video_port=True, quality=75)
#    img = io.BytesIO()
#    self.cam.capture(img,format='bgr')

    sys.stderr.write('Camera ready:\r\n')
    sys.stderr.write('  Exposure: %s\r\n' % (str(self.cam.exposure_speed)))
    sys.stderr.write('  ISO: %s\r\n' % (str(self.cam.iso)))
    sys.stderr.write('  Digital gain: %s\r\n' % (str(float(self.cam.digital_gain))))
    sys.stderr.write('  Analog gain: %s\r\n' % (str(float(self.cam.analog_gain))))

    self.seq_count = 0



  def nightshot(self):
    
#    img = io.BytesIO()
    imfn = './images/image_seq_%04d.%s.jpg' % (self.seq_count,str(time.time()))
    sys.stderr.write('  Capturing image: %s  \r\n' % (imfn))
    self.cam.capture(imfn, use_video_port=True, quality=75)
#    self.cam.capture(img,format='bgr')
    sys.stderr.write('      Done\r\n')

    #self.cam.capture_sequence(['./images/image%02d.jpg' % i for i in range(5)],quality=25)


  '''
  def sync_time(self):
    HOST = self.scopeserver_host
    PORT = self.scopeserver_port
    cmd = '{get_scopeserver_time}'

    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      sock.connect((HOST, PORT))
      sock.sendall(bytes(cmd, "utf-8"))
    
      # Receive response from the server and finish
      response = str(sock.recv(1024), "utf-8")

    if response.split()[0] == 'scopeserver_time':
      t_ss = int(response.split()[1])
      time.clock_settime_ns(time.CLOCK_REALTIME,t_ss)
      sys.stderr.write('Time Synchronized: %d\r\n' % (t_ss))
      sys.stderr.write('Current Time:  %s\r\n' % (time.strftime('%a %b %d %H:%M:%S %Z %Y')))
    else:
      self.scopeserver_connected = False
      sys.stderr.write('Could Not Synchronize Time\r\n')
  '''

  def sync_time(self):
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
        sock.sendall(bytes(cmd, "utf-8"))

        # Receive response from the server and finish
        response = str(sock.recv(1024), "utf-8")
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

