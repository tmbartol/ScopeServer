#!/usr/bin/env python3

import socket
import threading
import socketserver
import sys
import tty
import termios
import time
import re
import math
import numpy as np


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
#          data = str(self.request.recv(16), 'ascii').strip()
          data = self.request.recv(1024)
#          sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('ascii')))
          if data:
            # Process client command
            cmd = data.decode('ascii').strip()
            response = scope.process_cmd(cmd)
            if response:
              # Send response to client
#              sys.stderr.write('Server responding:  %s\r\n' % (response.encode('ascii')))
              self.request.sendall(response.encode('ascii'))
          else:
            return
      except ConnectionResetError:
        sys.stderr.write('Connection reset by peer.\r\n')
        pass
#      scope.server_thread.join()


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True
    timeout = None
    request_queue_size = 20


class scope_server:

  def __init__(self):
    self.input_running = False
    self.server_running = False
    self.ra_axis_running = False
    self.dec_axis_running = False
    self.ra_axis_guiding = False
    self.goto_target_running = False
    self.resume_mode = 'NONE'  # allowed functions:  NONE, GUIDE, GOTO
    self.process_cmd = self.process_lx200_cmd
    self.res_ra = 40000.0
    self.res_dec = 40000.0
    self.pos_ra = 10000.0
    self.pos_dec = 0.0
    self.dec_angle = self.get_dec_angle()
    self.ra_axis_start_pos = self.pos_ra
    self.delta = 1.0
    tropical_year = 365.242190402
#    self.sidereal_day = 86400*tropical_year/(1.0+tropical_year)
    self.sidereal_day = 86400.0/1.002737909350795
    self.sidereal_rate = self.res_ra/self.sidereal_day
    self.guide_rate = self.sidereal_rate
    self.slew_rate_max = self.res_ra/360.
    self.slew_rate_find = self.slew_rate_max/10
    self.slew_rate_center = self.slew_rate_max/50
    self.slew_rate_guide = self.guide_rate
    self.slew_rate = self.slew_rate_max

    self.timezone = time.timezone

    self.latitude = 32.0
    self.target_ra_pos = 0.0
    self.target_ra_time_array = [0,0,0]
    self.target_ra_radians = 0.0
    self.target_dec_pos = 0.0
    self.target_dec_radians = 0.0
    self.target_epsilon_pos = 1.0/2.0


  # Get declination angle position of scope
  def get_dec_angle(self):
    dec_angle = 360.0*self.pos_dec/self.res_dec
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
    pos = (self.res_dec*dec_angle/360.0)%self.res_dec
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
    pos = ((ra_time - time.time())*self.res_ra/86400.0)%self.res_ra
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


  # Start thread to run RA axis at guiding rate
  def ra_axis_guide_start(self, rate):
    self.guide_rate = rate
    self.ra_axis_guiding = True
    self.thread_ra_axis_guide = threading.Thread(target=self.ra_axis_guide)
    self.thread_ra_axis_guide.setDaemon(1)
    self.thread_ra_axis_guide.start()


  # Stop RA axis guiding
  def ra_axis_guide_stop(self,resume_mode='NONE'):
    if self.ra_axis_guiding:
      self.ra_axis_guiding=False
      self.resume_mode=resume_mode
      self.thread_ra_axis_guide.join()


  # Run RA axis at guiding rate
  def ra_axis_guide(self):
    self.ra_axis_guide_start_time = time.time()
    self.ra_axis_guide_start_pos = self.pos_ra
    while self.ra_axis_guiding:
      elapsed_t = time.time() - self.ra_axis_guide_start_time
      self.pos_ra = (self.ra_axis_guide_start_pos - elapsed_t*self.guide_rate)%self.res_ra
      time.sleep(0.01)


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
    


  # Start thread to slew RA axis
  def ra_axis_start(self, direction):
    self.ra_axis_running = True
    self.thread_ra_axis = threading.Thread(target=self.ra_axis_run, args=(direction,))
    self.thread_ra_axis.setDaemon(1)
    self.thread_ra_axis.start()


  # Stop slew of RA axis
  def ra_axis_stop(self):
    if self.ra_axis_running:
      self.ra_axis_running=False
      self.thread_ra_axis.join()


  # Slew RA axis
  def ra_axis_run(self, direction):
    self.ra_axis_start_time = time.time()
    self.ra_axis_start_pos = self.pos_ra
    while self.ra_axis_running:
      elapsed_t = time.time() - self.ra_axis_start_time
      self.pos_ra = (self.ra_axis_start_pos + elapsed_t*direction*self.slew_rate)%self.res_ra
      time.sleep(0.01)


  # Start thread to slew DEC axis
  def dec_axis_start(self, direction):
    self.dec_axis_running = True
    self.thread_dec_axis = threading.Thread(target=self.dec_axis_run, args=(direction,))
    self.thread_dec_axis.setDaemon(1)
    self.thread_dec_axis.start()


  # Stop slew of DEC axis
  def dec_axis_stop(self):
    if self.dec_axis_running:
      self.dec_axis_running = False
      self.thread_dec_axis.join()


  # Slew DEC axis
  def dec_axis_run(self, direction):
    self.dec_axis_start_time = time.time()
    self.dec_axis_start_pos = self.pos_dec
    while self.dec_axis_running:
      elapsed_t = time.time() - self.dec_axis_start_time
      self.pos_dec = (self.dec_axis_start_pos + elapsed_t*direction*self.slew_rate)%self.res_dec
      time.sleep(0.01)


  # Start server thread 
  def server_start(self,server_address,port):
    self.input_start()
    self.server_running = True

    self.threaded_server = ThreadedTCPServer((server_address, port), ThreadedTCPRequestHandler)

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    self.server_thread = threading.Thread(target=self.threaded_server.serve_forever)
    # Exit the server thread when the main thread terminates
    self.server_thread.daemon = True
    self.server_thread.start()
    sys.stderr.write('Waiting for a connection...\r\n')
    self.server_thread.join()


    '''
    self.thread_server = threading.Thread(target=self.server, args=(server_address, port,))
    self.thread_server.setDaemon(1)
    self.thread_server.start()
    self.thread_server.join()
    '''

    


  # Stop server thread
  def server_stop(self):
    if self.server_running:
      self.server_running=False
#      self.socket.shutdown(socket.SHUT_RDWR)
#      self.socket.close()
      sys.stderr.write('Scope Server shutting down.\r\n')
      self.threaded_server.shutdown()
      self.server_thread.join()


  # The server
  def server(self,server_address,port):
    sys.stderr.write('Scoper Server starting up on %s port %s\r\n' % (server_address, port))

    # Create a TCP/IP socket
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the address and port given
    self.socket.bind((server_address, port))
    self.socket.listen(1)
    sys.stderr.write('Waiting for a connection...\r\n')

    # Enter server loop
    while self.server_running:
#        sys.stderr.write('Waiting for a connection...\r\n')
        try:
          connection, client_address = self.socket.accept()
#          sys.stderr.write('Client connected: %s\r\n' % (str(client_address)))
          try:
            while self.server_running:
              # Receive data from client
              data = connection.recv(16)
#              sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('ascii')))
              if data:
                # Process client command
                cmd = data.decode('ascii').strip()
                response = scope.process_cmd(cmd)
                if response:
                  # Send response to client
                  connection.sendall(response.encode('ascii'))
              else:
                break
          finally:
            connection.close()

        # Handle reset by client
        except ConnectionResetError:
          sys.stderr.write('Connection reset by peer.\r\n')
          pass
        # Shutdown the server
        except:
          sys.stderr.write('Scope Server shutting down.\r\n')


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
      if k=='\x1b[A':
        self.pos_dec = (self.pos_dec + self.delta)%self.res_dec
        sys.stderr.write('  moved N to:  %.9g %.9g\r\n' % (self.pos_ra, self.pos_dec))
      elif k=='\x1b[B':
        self.pos_dec = (self.pos_dec - self.delta)%self.res_dec
        sys.stderr.write('  moved S to:  %.9g %.9g\r\n' % (self.pos_ra, self.pos_dec))
      elif k=='\x1b[C':
        if self.ra_axis_running:
          self.ra_axis_start_pos = (self.ra_axis_start_pos - self.delta)%self.res_ra
        else:
          self.pos_ra = (self.pos_ra - self.delta)%self.res_ra
        sys.stderr.write('  moved E to:  %.9g %.9g\r\n' % (self.pos_ra, self.pos_dec))
      elif k=='\x1b[D':
        if self.ra_axis_running:
          self.ra_axis_start_pos = (self.ra_axis_start_pos + self.delta)%self.res_ra
        else:
          self.pos_ra = (self.pos_ra + self.delta)%self.res_ra
        sys.stderr.write('  moved W to:  %.9g %.9g\r\n' % (self.pos_ra, self.pos_dec))

      # Start RA axis guiding
      elif k=='g':
        sys.stderr.write('  Start Guiding RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
        self.ra_axis_guide_start(self.guide_rate)

      # Stop RA axis guiding
      elif k=='s':
        self.ra_axis_guide_stop(resume_mode='NONE')
        sys.stderr.write('  Stopped Guiding RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Shutdown the server
      elif k=='q':
        self.input_running = False
      else:
        sys.stderr.write('Unknown key! Please type q to quit.\r\n')
    self.server_stop()


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

    elif cmd[0:3] == ':Sg':  # Set site latitude: DDD*MM#
      ddd, mm = cmd[3:-1].split('*')
      self.latitude = float(ddd) + float(mm)/60.0
      response = '1'

    elif cmd[0:3] == ':St':  # Set site latitude:  sDD*MM#
      dd, mm = cmd[3:-1].split('*')
      self.latitude = float(dd) + float(mm)/60.0
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
      self.set_target_ra_time_array(cmd[3:-1])
      self.target_ra_radians = self.ra_time_to_radians(cmd[3:-1])
      response = '1'

    elif cmd[0:3] == ':Sd':  # Set target object Dec: sDD*MM:SS#
      self.target_dec_pos = self.dec_angle_to_pos(cmd[3:-1])
      self.target_dec_radians = self.dec_angle_to_radians(cmd[3:-1])
      response = '1'

    elif cmd == ':MS#':  # Slew to target
      response = '0'
      if self.ra_axis_guiding:
        self.ra_axis_guide_stop(resume_mode='GUIDE')
      self.ra_axis_stop()
      self.dec_axis_stop()
      self.goto_target_start()

    elif cmd == ':Mn#':  # Start move North
      self.dec_axis_stop()
      self.goto_target_stop(resume_mode='GOTO')
      self.dec_axis_start(1.0)

    elif cmd == ':Ms#':  # Start move South
      self.dec_axis_stop()
      self.goto_target_stop(resume_mode='GOTO')
      self.dec_axis_start(-1.0)

    elif cmd == ':Me#':  # Start move East
      if self.ra_axis_guiding:
        self.ra_axis_guide_stop(resume_mode='GUIDE')
      elif self.goto_target_running:
        self.goto_target_stop(resume_mode='GOTO')
      self.ra_axis_stop()
      self.ra_axis_start(1.0)

    elif cmd == ':Mw#':  # Start move West
      if self.ra_axis_guiding:
        self.ra_axis_guide_stop(resume_mode='GUIDE')
      elif self.goto_target_running:
        self.goto_target_stop(resume_mode='GOTO')
      self.ra_axis_stop()
      self.ra_axis_start(-1.0)

    elif cmd == ':Qn#':  # Stop move North
      self.dec_axis_stop()
      if self.resume_mode=='GOTO':
        self.goto_target_start()

    elif cmd == ':Qs#':  # Stop move South
      self.dec_axis_stop()
      if self.resume_mode=='GOTO':
        self.goto_target_start()

    elif cmd == ':Qe#':  # Stop move East
      self.ra_axis_stop()
      if self.resume_mode=='GUIDE':
        self.ra_axis_guide_start(self.guide_rate)
      elif self.resume_mode=='GOTO':
        self.goto_target_start()

    elif cmd == ':Qw#':  # Stop move West
      self.ra_axis_stop()
      if self.resume_mode=='GUIDE':
        self.ra_axis_guide_start(self.guide_rate)
      elif self.resume_mode=='GOTO':
        self.goto_target_start()

    elif cmd == ':Q#':  # Stop all slews
      self.ra_axis_stop()
      self.dec_axis_stop()
      if self.ra_axis_guiding:
        self.ra_axis_guide_stop(resume_mode='GUIDE')
      elif self.goto_target_running:
        self.goto_target_stop(resume_mode='GOTO')
#      if self.resume_mode=='GUIDE':
#        self.ra_axis_guide_start(self.guide_rate)
#      if self.resume_mode=='GOTO':
#        self.goto_target_start()

    else:
      pass
  
    return response


if (__name__ == '__main__'):

  if (len(sys.argv)<3):
    print('\nUsage: %s server_address port\n' % (sys.argv[0]))
    print('   Example: %s 10.0.1.12 4030\n' % (sys.argv[0]))
    sys.exit()

  server_address = sys.argv[1]
  port = int(sys.argv[2])
  scope = scope_server()
  scope.server_start(server_address, port)

