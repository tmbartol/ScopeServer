#!/usr/bin/env python3

import socket
import threading
import socketserver
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

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pyPicServo import nmccom


# Class to get raw characters from terminal input
class _Getch:
  def __call__(self):
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
      while scope.server_running:
      # Receive data from client
        data = self.request.recv(1024)
#        sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('ascii')))
        if data:
          # Process client command
          cmd = data.decode('ascii').strip()
          if cmd:
            if cmd[0] == ':':
              response = scope.process_lx200_cmd(cmd)
            if cmd[0] == '{':
              response = scope.process_scopeserver_cmd(cmd)
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


class scope_server:

  def __init__(self, scope_mode = 'REAL_SCOPE'):
    self.scope_mode = scope_mode  # allowed: 'SIM_SCOPE'  or  'REAL_SCOPE'
    self.input_running = False
    self.server_running = False
    self.motion_q = queue.Queue(maxsize=0)
    self.motion_running = False
    self.ra_axis_running = False
    self.dec_axis_running = False
    self.ra_axis_goto = False
    self.ra_axis_guiding = False
    self.goto_target_running = False
    self.resume_mode = 'NONE'  # allowed functions:  NONE, GUIDE, GOTO
    self.encoder_res_ra = 40000
    self.encoder_res_dec = 40000
    self.worm_gear_ra = 576
    self.worm_gear_dec = 450
    self.res_ra = self.worm_gear_ra*self.encoder_res_ra
    self.res_dec = self.worm_gear_dec*self.encoder_res_dec
    self.degree_counts_ra=int(self.res_ra/360)
    self.degree_counts_dec=int(self.res_dec/360)
    self.pos_ra = 90*self.degree_counts_ra  # home RA angle is 90 degrees
    self.pos_dec = 0*self.degree_counts_dec  # home Dec angle is 0 degrees
    self.dec_angle = self.get_dec_angle()
    self.ra_axis_start_pos = self.pos_ra
    tropical_year = 365.242190402
#    self.sidereal_day = 86400*tropical_year/(1.0+tropical_year)
    self.sidereal_day = 86400.0/1.002737909350795
    self.sidereal_rate = self.res_ra/self.sidereal_day
    self.guide_rate = self.sidereal_rate
    self.delta = 1.0*self.guide_rate
    sys.stderr.write('\n\nGuide rate set to: %.6g  counts per second\n\n' % (self.guide_rate))
    self.slew_rate_max = 2*self.res_ra/360.
    self.slew_rate = self.slew_rate_max
    self.slew_rate_find = self.slew_rate_max/10
    self.slew_rate_center = self.slew_rate_max/50
    self.slew_rate_guide = self.guide_rate

    self.timezone = time.timezone

    try:
      import gpsd
      gpsd.connect()
      lat, lon = gpsd.get_current().position()
      self.site_latitude = lat
      self.site_longitude = lon
    except ImportError:
      self.site_latitude = 33.0831
      self.site_longitude = -117.2455

    sys.stderr.write('\nStarting Scope Sever in %s mode\n' % (self.scope_mode))
    sys.stderr.write('    Site location set to: Lat: %.9g   Lon: %.9g\n\n' % (self.site_latitude, self.site_longitude))

    self.target_ra_pos = 0.0
    self.target_ra_time = '00:00:00'
    self.target_ra_time_array = [0,0,0]
    self.target_dec_pos = 0.0
    self.target_dec_angle = "+00*00'00"
#    self.target_epsilon_pos = 1.0/2.0
    self.target_epsilon_pos = 300.0
 
    if self.scope_mode == 'REAL_SCOPE':
      nmc_net = nmccom.NmcNet()
      nmc_net.Initialize(['RA','Dec'],baudrate=230400)
      self.ra_mod = nmc_net.modules['RA']
      self.dec_mod = nmc_net.modules['Dec']
      self.ra_mod.verbosity = 1
      self.dec_mod.verbosity = 1
      self.servo_sidereal_rate = int(self.sidereal_rate*0.000512*2**16)
      self.servo_ra_slew_rate = int(self.slew_rate*0.000512*2**16)
      self.servo_dec_slew_rate = int(self.slew_rate*0.000512*2**16)
 
      self.ra_mod.ServoIOControl(output_mode=nmccom.PH3_MODE)
      self.dec_mod.ServoIOControl(output_mode=nmccom.PH3_MODE)
      self.ra_mod.ServoSetGain(200, 800, 200, 100, 255, 0, 4000, 1, 0, 1)
      self.dec_mod.ServoSetGain(200, 800, 200, 100, 255, 0, 4000, 1, 0, 1)
      self.ra_mod.ServoSetPos(int(self.pos_ra))
      self.dec_mod.ServoSetPos(int(self.pos_dec))

      # Example nmccom commands
      '''
      # Servo On
      #self.ra_mod.ServoStopMotor()
      #self.dec_mod.ServoStopMotor()

      # Servo Off
      #self.ra_mod.ServoStopMotorOff()
      #self.dec_mod.ServoStopMotorOff()

      # Slew RA at fast rate
      self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 15*self.degree_counts_ra, self.servo_fast_rate, 400, 0)
      self.ra_mod.NoOp()
      while not self.ra_mod.response[0] & 0x01:
        if i%200 == 0:
          self.ra_mod.PrintFullStatusReport()
        self.ra_mod.NoOp()
        i+=1


      # Slew RA at sidereal rate
      # Servo On
      #self.ra_mod.ServoStopMotor()
      pos = self.ra_mod.ServoGetPos()
      self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, pos + 360*self.degree_counts_ra, servo_sidereal_rate, 100, 0)
      self.ra_mod.NoOp()
      while not self.ra_mod.response[0] & 0x01:
        if i%200 == 0:
          self.ra_mod.PrintFullStatusReport()
        self.ra_mod.NoOp()
        i+=1
      '''


  def get_status(self):
    status_dict = {}

    status_dict['site_local_time'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    status_dict['site_utc_time'] = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
    status_dict['site_latitude'] = self.site_latitude
    status_dict['site_longitude'] = self.site_longitude

    status_dict['dec_angle'] = self.get_dec_angle()
    status_dict['dec_pos'] = self.pos_dec
    status_dict['ra_time'] = self.get_ra_time()
    status_dict['ra_pos'] = self.pos_ra

    status_dict['target_dec_angle'] = self.target_dec_angle
    status_dict['target_dec_pos'] = self.target_dec_pos
    status_dict['target_ra_time'] = self.target_ra_time
    status_dict['target_ra_pos'] = self.ra_time_array_to_pos(self.target_ra_time_array)

    return status_dict


  # Get declination angle position of scope
  def get_dec_angle(self):
    dec_angle = 360.0*(-self.pos_dec%self.res_dec)/self.res_dec
    dec_dd = int(dec_angle)
    dec_rem = 60*abs(dec_angle - dec_dd)
    dec_mm = int(dec_rem)
    dec_ss = int(60*abs(dec_rem - dec_mm))
    return ("%+.2d*%.2d'%.2d" % (dec_dd, dec_mm, dec_ss))
#    return ('%+.2d*%.2d:%.2d' % (dec_dd, dec_mm, dec_ss))
#    return ('%+.2d*%.2d' % (dec_dd, dec_mm))
    

  # Convert Dec angle string to shaft pos
  def dec_angle_to_pos(self,dec_angle_str):
    dd = (dec_angle_str.split('*')[0])
    dd_sign = float(dec_angle_str[0] + '1')
    mm,ss = (dec_angle_str.split('*')[1]).split(':')
    dd = float(dd)
    mm = float(mm)
    ss = float(ss)
    dec_angle = dd + (dd_sign*(mm/60.0 + ss/3600.0))
    pos = -dd_sign*((self.res_dec*dec_angle/360.0)%self.res_dec)
    sys.stderr.write('Target Dec Angle: %s  pos %.9g\r\n' % (dec_angle_str, pos))
    return (pos)
    

  # Convert Dec angle string to radians
  def dec_angle_to_radians(self,dec_angle_str):
    dd = (dec_angle_str.split('*')[0])
    mm,ss = (dec_angle_str.split('*')[1]).split(':')
    dec_angle = float(dd) + float(mm)/60.0 + float(ss)/3600.0
    dec_angle_radians = dec_angle*math.pi/180.0
    return (dec_angle_radians)
    

  # Get RA time position of scope
  def get_ra_time(self):
    # In a moving reference frame with rotation of Earth:
    ra_time = time.time() + 86400.0*self.pos_ra/self.res_ra
    ra_time_str = time.ctime(ra_time).split()[3]
    return (ra_time_str)
#    return ('%.2d:%.2d.0' % (ra_hh, ra_mm))


  def set_target_ra_time_array(self,ra_time_str):
    ra_time = ra_time_str.split(':')
    self.target_ra_time_array = []
    self.target_ra_time_array.append(int(ra_time[0]))
    self.target_ra_time_array.append(int(ra_time[1]))
    self.target_ra_time_array.append(int(ra_time[2]))
    pos = self.ra_time_array_to_pos(self.target_ra_time_array)
    sys.stderr.write('Target RA: %s  pos %.9g\r\n' % (ra_time_str, pos))
    return


  # Convert RA time string to shaft pos
  def ra_time_array_to_pos(self,ra_time_array):
    # In a moving reference frame with rotation of Earth:
    tt = list(time.localtime(time.time()))
    tt[3:6] = ra_time_array[:]
    ra_time = time.mktime(tuple(tt))
    dst = 0*3600.0  # Correct for Daylight Savings Time
    pos = ((ra_time - time.time() - dst)*self.res_ra/86400.0)%self.res_ra
#    sys.stderr.write('target ra_time: %02d:%02d:%02d  pos %.9g\r\n' % (ra_time_array[0], ra_time_array[1], ra_time_array[2], pos))
    return (pos)


  # Convert RA time string to radians
  def ra_time_to_radians(self,ra_time_str):
    hh,mm,ss = ra_time_str.split(':')
    secs = 3600.0*float(hh)+60.0*float(mm)+float(ss)
    ra_angle_radians = 2*math.pi*secs/86400.0
    return (ra_angle_radians)


  # Start thread to get terminal input
  def input_start(self):
    self.input_running = True
    self.thread_get_input = threading.Thread(target=self.get_input)
    self.thread_get_input.setDaemon(1)
    self.thread_get_input.start()


##################

  # Start thread for motion control
  def motion_control_start(self):
    self.thread_motion_control = threading.Thread(target=self.motion_control)
    self.thread_motion_control.setDaemon(1)
    self.thread_motion_control.start()


  def motion_control(self):
    sys.stderr.write('motion_control: thread starting...\n')
    while True:
      if self.motion_running:
        # if in motion get motion command but do not wait if queue is empty
        try:
          motion_cmd = self.motion_q.get_nowait()
        except queue.Empty:
          motion_cmd = ['motion_continue', None]
      else:
        # if not in motion wait to receive motion command
        sys.stderr.write('motion_control: idle, waiting for command\n')
        motion_cmd = self.motion_q.get()

      cmd = motion_cmd[0]
      cmd_arg = motion_cmd[1]
      if cmd == 'ra_slew_start':
        sys.stderr.write('motion_control: RA slew start\n')
        # RA Slew at slew rate
        self.ra_axis_running = True
        # RA Servo On
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.ra_axis_start_pos = self.pos_ra
        self.ra_axis_start_time = time.time()
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, cmd_arg*180*self.degree_counts_ra, self.servo_ra_slew_rate, 400, 0)

      elif cmd == 'ra_slew_stop':
        sys.stderr.write('motion_control: RA slew stop\n')
        # RA Smooth Stop
        self.ra_mod.ServoStopSmooth()

      elif cmd == 'ra_guide_start':
        sys.stderr.write('motion_control: RA guide start\n')
        # Slew RA at sidereal rate
        self.ra_axis_guiding = True
        # Servo On
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.ra_axis_guide_start_pos = self.pos_ra
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -180*self.degree_counts_ra, self.servo_sidereal_rate, 100, 0)

      elif cmd == 'ra_guide_stop':
        sys.stderr.write('motion_control: RA guide stop\n')
        # RA Smooth Stop
        self.ra_mod.ServoStopSmooth()

      elif cmd == 'dec_slew_start':
        sys.stderr.write('motion_control: Dec slew start\n')
        # Dec Slew at slew rate
        self.dec_axis_running = True
        # Dec Servo On
        self.dec_mod.ServoStopMotor()
        self.pos_dec = self.dec_mod.ServoGetPos()
        self.dec_axis_start_pos = self.pos_dec
        self.dec_axis_start_time = time.time()
        self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, cmd_arg*180*self.degree_counts_dec, self.servo_dec_slew_rate, 400, 0)

      elif cmd == 'dec_slew_stop':
        sys.stderr.write('motion_control: Dec slew stop\n')
        # Dec Smooth Stop
        self.dec_mod.ServoStopSmooth()

      elif cmd == 'goto_target_start':
        sys.stderr.write('motion_control: GOTO target start\n')
        self.target_ra_pos = self.ra_time_array_to_pos(self.target_ra_time_array)
        ra_distance = self.target_ra_pos - self.pos_ra
        ra_dir = 1.0-2.0*(ra_distance<0)
        if abs(ra_distance) > self.res_ra/2.0:
          ra_distance = -ra_dir*(self.res_ra-abs(ra_distance))
          ra_dir = -ra_dir

        dec_distance = self.target_dec_pos - self.pos_dec
        dec_dir = 1.0-2.0*(dec_distance<0)
        if abs(dec_distance) > self.res_dec/2.0:
          dec_distance = -dec_dir*(self.res_dec-abs(dec_distance))
          dec_dir = -dec_dir

        target_distance = math.sqrt(ra_distance**2 + dec_distance**2)
        ra_speed = abs(ra_distance/target_distance)
        dec_speed = abs(dec_distance/target_distance)
        servo_ra_goto_rate = int(ra_speed*self.slew_rate*0.000512*2**16)
        servo_dec_goto_rate = int(dec_speed*self.slew_rate*0.000512*2**16)

        # RA GOTO Target at goto rate
        self.ra_axis_running = True
        self.ra_axis_goto = True
        # RA Servo On
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.ra_axis_start_pos = self.pos_ra
        self.ra_axis_start_time = time.time()

        # Dec GOTO Target at goto rate
        self.dec_axis_running = True
        # Dec Servo On
        self.dec_mod.ServoStopMotor()
        self.pos_dec = self.dec_mod.ServoGetPos()
        self.dec_axis_start_pos = self.pos_dec
        self.dec_axis_start_time = time.time()

        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), servo_ra_goto_rate, 400, 0)
        self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_dec_pos), servo_dec_goto_rate, 400, 0)

      elif cmd == 'goto_target_stop':
        sys.stderr.write('motion_control: GOTO target stop\n')

      elif cmd == 'motion_continue':
        pass

      if self.ra_axis_goto:
        # Update RA GOTO Target
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.target_ra_pos = self.ra_time_array_to_pos(self.target_ra_time_array)
        if abs(self.pos_ra - self.target_ra_pos) > self.target_epsilon_pos:
          self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), 0)

      if self.ra_axis_running or self.ra_axis_guiding:
        self.pos_ra = self.ra_mod.ServoGetPos()
        if self.ra_mod.response[0] & 0x01:
          if self.ra_axis_goto:
            self.motion_q.put(['ra_guide_start', None])
          # Servo Off
          self.ra_mod.ServoStopMotorOff()
          self.ra_axis_running = False
          self.ra_axis_guiding = False
          self.ra_axis_goto = False

      if self.dec_axis_running:
        self.pos_dec = self.dec_mod.ServoGetPos()
        if self.dec_mod.response[0] & 0x01:
          # Servo Off
          self.dec_mod.ServoStopMotorOff()
          self.dec_axis_running = False

      self.motion_running = self.ra_axis_running or self.ra_axis_guiding or self.dec_axis_running
      if not cmd == 'motion_continue':
        self.motion_q.task_done()

##################

  # Start thread to goto target
  def goto_target_start(self):
    self.goto_target_running = True
    self.thread_goto_target = threading.Thread(target=self.goto_target_run,)
    self.thread_goto_target.setDaemon(1)
    self.thread_goto_target.start()


  # Stop slew to goto target
  def goto_target_stop(self, resume_mode='NONE'):
    if self.goto_target_running:
      self.goto_target_running=False
      self.resume_mode=resume_mode
      self.thread_goto_target.join()

  def goto_target_run(self):
    sys.stderr.write('Executing GOTO...\r\n')

    target_ra_pos = self.ra_time_array_to_pos(self.target_ra_time_array)
    ra_distance = target_ra_pos - self.pos_ra
    ra_dir = 1.0-2.0*(ra_distance<0)
    if abs(ra_distance) > self.res_ra/2.0:
      ra_distance = -ra_dir*(self.res_ra-abs(ra_distance))
      ra_dir = -ra_dir

    dec_distance = self.target_dec_pos - self.pos_dec
    dec_dir = 1.0-2.0*(dec_distance<0)
    if abs(dec_distance) > self.res_dec/2.0:
      dec_distance = -dec_dir*(self.res_dec-abs(dec_distance))
      dec_dir = -dec_dir

    target_distance = math.sqrt(ra_distance**2 + dec_distance**2)
    ra_speed = abs(ra_distance/target_distance)
    dec_speed = abs(dec_distance/target_distance)
    
    while self.goto_target_running and (target_distance > self.target_epsilon_pos):
      self.pos_ra = (self.pos_ra + 0.01*ra_dir*ra_speed*self.slew_rate)%self.res_ra
      self.pos_dec = (self.pos_dec + 0.01*dec_dir*dec_speed*self.slew_rate)%self.res_dec
      target_ra_pos = self.ra_time_array_to_pos(self.target_ra_time_array)
      ra_distance = target_ra_pos - self.pos_ra
      ra_dir = 1.0-2.0*(ra_distance<0)
      if abs(ra_distance) > self.res_ra/2.0:
        ra_distance = -ra_dir*(self.res_ra-abs(ra_distance))
        ra_dir = -ra_dir

      dec_distance = self.target_dec_pos - self.pos_dec
      dec_dir = 1.0-2.0*(dec_distance<0)
      if abs(dec_distance) > self.res_dec/2.0:
        dec_distance = -dec_dir*(self.res_dec-abs(dec_distance))
        dec_dir = -dec_dir

      target_distance = math.sqrt(ra_distance**2 + dec_distance**2)
      ra_speed = abs(ra_distance/target_distance)
      dec_speed = abs(dec_distance/target_distance)
      time.sleep(0.01)

    self.goto_target_running = False
    sys.stderr.write('Completed GOTO...\r\n')
    sys.stderr.write('Entering guide mode...\r\n')
    self.ra_axis_guide_start(self.guide_rate)
#    self.thread_ra_axis_guide.join()
    sys.stderr.write('Returning from GOTO thread\r\n')


  # Start server thread 
  def server_start(self,server_address,port):
    self.input_start()
    self.motion_control_start()
    self.server_running = True

    self.threaded_server = ThreadedTCPServer((server_address, port), ThreadedTCPRequestHandler)

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    self.server_thread = threading.Thread(target=self.threaded_server.serve_forever)
    # Exit the server thread when the main thread terminates
    self.server_thread.daemon = True
    self.server_thread.start()
    sys.stderr.write('Waiting for a connection, type q to quit...\r\n')
    self.server_thread.join()


  # Stop server thread
  def server_stop(self):
    if self.server_running:
      self.server_running=False
#      self.socket.shutdown(socket.SHUT_RDWR)
#      self.socket.close()
      sys.stderr.write('Scope Server shutting down.\r\n')
      self.threaded_server.shutdown()
      self.server_thread.join()


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
      # N, up arrow
      if k=='\x1b[A':
        if self.dec_axis_running:
          self.motion_q.put(['dec_slew_stop', None])
        else:
          self.motion_q.put(['dec_slew_start', 1])
#        sys.stderr.write('  moved N to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # S, down arrow
      elif k=='\x1b[B':
        if self.dec_axis_running:
          self.motion_q.put(['dec_slew_stop', None])
        else:
          self.motion_q.put(['dec_slew_start', -1])
#        sys.stderr.write('  moved S to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # E, right arrow
      elif k=='\x1b[C':
        if self.ra_axis_running:
          self.motion_q.put(['ra_slew_stop', None])
        elif self.ra_axis_guiding:
          self.motion_q.put(['ra_slew_start', 1])
        else:
          self.motion_q.put(['ra_slew_start', 1])
#        sys.stderr.write('  moved E to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # W, left arrow
      elif k=='\x1b[D':
        if self.ra_axis_running:
          self.motion_q.put(['ra_slew_stop', None])
        elif self.ra_axis_guiding:
          self.motion_q.put(['ra_slew_start', -1])
        else:
          self.motion_q.put(['ra_slew_start', -1])
#        sys.stderr.write('  moved W to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Start RA axis guiding
      elif k=='g':
        if not self.ra_axis_guiding:
          sys.stderr.write('  Start Guiding RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
          self.motion_q.put(['ra_guide_start', None])

      # Stop RA axis guiding
      elif k=='s':
        if self.ra_axis_guiding:
          self.motion_q.put(['ra_guide_stop', None])
          sys.stderr.write('  Stopped Guiding RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Report Position
      elif k=='p':
        sys.stderr.write('  Current Pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Shutdown the server
      elif k=='q':
        self.input_running = False
      else:
        sys.stderr.write('Unknown key! Please type q to quit.\r\n')
    self.server_stop()


  # Process ScopeSever Telescope Protocol Commands
  def process_scopeserver_cmd(self,cmd):

    response = None

    cmd = cmd[1:-1]
    if cmd == 'get_status':
      response = str(self.get_status())

    elif cmd == 'shutdown_server':
      self.server_stop()

    elif cmd == 'shutdown_system':
      pass

    else:
      pass

    return response


  # Process LX200 Telescope Protocol Commands
  def process_lx200_cmd(self,cmd):

    response = None

    if (cmd != ':GD#') & (cmd != ':GR#'):
      sys.stderr.write('  Processing LX200 cmd: %s\r\n' % (cmd))
#    sys.stderr.write('  Processing LX200 cmd: %s\r\n' % (cmd))

    if cmd == ':GD#':  # Get Dec angle:  sDD*MM'SS#
      response = '%s#' % self.get_dec_angle()
#      sys.stderr.write('  Server responding: %s\r\n' % (response))

    elif cmd == ':GR#': # Get RA time:  HH:MM:SS#
      response = '%s#' % self.get_ra_time()
#      sys.stderr.write('  Server responding: %s\r\n' % (response))

    elif cmd[0:3] == ':St':  # Set site latitude:  sDD*MM#
      dd, mm = cmd[3:-1].split('*')
      dd_sign = float(dd[0] + '1')
      self.site_latitude = float(dd) + dd_sign*float(mm)/60.0
      response = '1'

    elif cmd[0:3] == ':Sg':  # Set site longitude: DDD*MM#
      ddd, mm = cmd[3:-1].split('*')
      self.site_longitude = float(ddd) + float(mm)/60.0
      response = '1'

    elif cmd[0:3] == ':SG':  # Set UTC offset: sHH.H#
      self.UTC_offset = float(cmd[3:-1])
      response = '1'

    elif cmd[0:3] == ':SL':  # Set local time: HH:MM:SS#
      self.local_time = cmd[3:-1]
      response = '1'

    elif cmd[0:3] == ':SC':  # Set local date:  MM/DD/YY#
      self.date = cmd[3:-1]
      response = '1"Updating Planetary Data# #"'

    elif cmd == ':CM#':
      response = "M31 EX GAL MAG 3.5 SZ178.0'#"

    elif cmd == ':RS#':
      self.slew_rate = self.slew_rate_max

    elif cmd == ':RM#':
      self.slew_rate = self.slew_rate_find

    elif cmd == ':RC#':
      self.slew_rate = self.slew_rate_center

    elif cmd == ':RG#':
      self.slew_rate = self.slew_rate_guide

    elif cmd[0:3] == ':Sr':  # Set target object RA:  HH:MM:SS#
      self.target_ra_time = cmd[3:-1]
      self.set_target_ra_time_array(cmd[3:-1])
      response = '1'

    elif cmd[0:3] == ':Sd':  # Set target object Dec: sDD*MM:SS#
      self.target_dec_angle = cmd[3:-1]
      self.target_dec_pos = self.dec_angle_to_pos(cmd[3:-1])
      response = '1'

    elif cmd == ':MS#':  # Slew to target
      response = '0'
#      if self.ra_axis_guiding:
#        self.ra_axis_guide_stop(resume_mode='GUIDE')
#      self.ra_axis_stop()
#      self.dec_axis_stop()
#      self.goto_target_start()
      self.motion_q.put(['goto_target_start',None])

    elif cmd == ':Mn#':  # Start move North
#      self.dec_axis_stop()
#      self.goto_target_stop(resume_mode='GOTO')
#      self.dec_axis_start(1.0)
      self.motion_q.put(['dec_slew_start',-1])

    elif cmd == ':Ms#':  # Start move South
#      self.dec_axis_stop()
#      self.goto_target_stop(resume_mode='GOTO')
#      self.dec_axis_start(-1.0)
      self.motion_q.put(['dec_slew_start',1])

    elif cmd == ':Me#':  # Start move East
#      if self.ra_axis_guiding:
#        self.ra_axis_guide_stop(resume_mode='GUIDE')
#      elif self.goto_target_running:
#        self.goto_target_stop(resume_mode='GOTO')
#      self.ra_axis_stop()
#      self.ra_axis_start(1.0)
      self.motion_q.put(['ra_slew_start',1])

    elif cmd == ':Mw#':  # Start move West
#      if self.ra_axis_guiding:
#        self.ra_axis_guide_stop(resume_mode='GUIDE')
#      elif self.goto_target_running:
#        self.goto_target_stop(resume_mode='GOTO')
#      self.ra_axis_stop()
#      self.ra_axis_start(-1.0)
      self.motion_q.put(['ra_slew_start',-1])

    elif cmd == ':Qn#':  # Stop move North
#      self.dec_axis_stop()
#      if self.resume_mode=='GOTO':
#        self.goto_target_start()
      self.motion_q.put(['dec_slew_stop', None])

    elif cmd == ':Qs#':  # Stop move South
#      self.dec_axis_stop()
#      if self.resume_mode=='GOTO':
#        self.goto_target_start()
      self.motion_q.put(['dec_slew_stop', None])

    elif cmd == ':Qe#':  # Stop move East
#      self.ra_axis_stop()
#      if self.resume_mode=='GUIDE':
#        self.ra_axis_guide_start(self.guide_rate)
#      elif self.resume_mode=='GOTO':
#        self.goto_target_start()
      self.motion_q.put(['ra_slew_stop', None])

    elif cmd == ':Qw#':  # Stop move West
#      self.ra_axis_stop()
#      if self.resume_mode=='GUIDE':
#        self.ra_axis_guide_start(self.guide_rate)
#      elif self.resume_mode=='GOTO':
#        self.goto_target_start()
      self.motion_q.put(['ra_slew_stop', None])

    elif cmd == ':Q#':  # Stop all slews
#      self.ra_axis_stop()
#      self.dec_axis_stop()
#      if self.ra_axis_guiding:
#        self.ra_axis_guide_stop(resume_mode='GUIDE')
#      elif self.goto_target_running:
#        self.goto_target_stop(resume_mode='GOTO')
      self.motion_q.put(['ra_slew_stop', None])
      self.motion_q.put(['dec_slew_stop', None])

    else:
      pass
  
    return response


if (__name__ == '__main__'):

  if (len(sys.argv)<3):
    print('\nUsage: %s server_address port\n' % (sys.argv[0]))
    print('   Example: %s 10.0.1.15 4030\n' % (sys.argv[0]))
    sys.exit()

  server_address = sys.argv[1]
  port = int(sys.argv[2])
  scope = scope_server()
  scope.server_start(server_address, port)
