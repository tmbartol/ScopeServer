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
from scipy.interpolate import UnivariateSpline
# from astropy.time import Time
import astropy.units as u
import os
import queue
import subprocess

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pyPicServo import nmccom


############################
# Shutdown Signal Handler
############################

def signal_handler(signal, frame):
  global nmc_net
  global fd, old_settings
  try:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  except:
    pass
  sys.stderr.write('\r\nScoperServer caught signal %d\r\n' % (signal))
  sys.stderr.write('ScopeServer shutting down...\r\n')
  nmc_net.modules['RA'].ServoStopMotorOff()
  nmc_net.modules['Dec'].ServoStopMotorOff()

  # Shutdown NMC Network
  nmc_net.Shutdown()

  sys.exit(0)


###########################
# Get IP address of Self
###########################

import netifaces as ni
def get_ip(iface = 'wlan0'):
  try:
    ni.ifaddresses(iface)
    ip = ni.ifaddresses(iface)[ni.AF_INET][0]['addr']
  except:
    ip = '127.0.0.1'
  return ip


####################################################
# Class to get raw characters from terminal input
####################################################

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


###########################################
# Classes for Threaded TCP Socket Server
###########################################

class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):
  def handle(self):
    try:
      while scope.server_running:
        # Receive data from client
        data = self.request.recv(1024)
#        sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('utf-8')))
        if data:
          # Process client command
          cmd = data.decode('utf-8').strip()
          if cmd:
            if cmd[0] == ':':
              response = scope.process_lx200_cmd(cmd)
            if cmd[0] == '{':
              response = scope.process_scopeserver_cmd(cmd)
            if response:
              # Send response to client
#              sys.stderr.write('Server responding:  %s\r\n' % (response.encode('utf-8')))
              self.request.sendall(response.encode('utf-8'))
        else:
          self.request.close()
          return
    except Exception as e:
      sys.stderr.write('Exception in ScopeServer TCP Command Processor.\r\n')
      '''
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
#  allow_reuse_address = True
#  daemon_threads = True
#  timeout = None
#  request_queue_size = 100
  pass


class SiderealTime():

  def __init__(self, t=None, longitude=None):
    if t == None:
      self.t = datetime.datetime.utcnow()
    else:
      self.t = t
    if longitude == None:
      self.longitude = 0.0
    else:
      self.longitude = longitude
    self.update_sidereal_time(t=self.t)

  def update_sidereal_time(self, t=None, longitude=None):
    if t == None:
      self.t = datetime.datetime.utcnow()
    else:
      self.t = t
    if longitude != None:
      self.longitude = longitude

    y,m,d = self.t.year, self.t.month, self.t.day
    j0 = 367*y - int(7*(y+int(m+9)/12)/4) - int(3*(int((y+(m-9)/7)/100)+1)/4) + int(275*m/9) + d + 1721028.5
    T0 = (j0-2451545)/36525.
    theta_G0 = (360/86400*(24110.5493771 + 8640184.794536*T0 + 0.0931118*T0**2 - 0.0000062*T0**3 + 0.0000013*T0**4))%360
    d_frac = (3600*self.t.hour + 60*self.t.minute + self.t.second + 1e-6*self.t.microsecond)/86400.
    theta_G = theta_G0 + 360.9856122880876*d_frac
    self.ERA = (theta_G + self.longitude)%360
    h = np.modf(self.ERA/15.)[1]
    m = np.modf(np.modf(self.ERA/15.)[0]*60)[1]
    s = np.modf(np.modf(self.ERA/15.)[0]*60)[0]*60
    self.ra_time_tuple = (h, m, s)
#    self.ra_time_str = '%02gh%02gm%07.4fs' % (h, m, s)
    self.ra_time_str = '%02d:%02d:%02d' % (int(h), int(m), int(s))



###############################
# Main Class for ScopeServer
###############################

class scope_server:

  def __init__(self, server_address, port, scope_mode = 'REAL_SCOPE'):
    global nmc_net
    self.scope_mode = scope_mode  # allowed: 'SIM_SCOPE'  or  'REAL_SCOPE'
    self.verbose = True
    self.input_running = False
    self.server_running = False
    self.motion_q = queue.Queue(maxsize=0)
    self.motion_running = False

    self.scopeserver_host = server_address
    self.scopeserver_port = port
    host_dict = {}
    host_dict['10.0.1.13'] = '10.0.1.14'
    host_dict['10.0.1.20'] = '10.0.1.23'
    host_dict['10.0.1.23'] = '10.0.1.24'
    host_dict['192.168.50.5'] = '192.168.50.10'
    self.autoguider_host = host_dict[self.scopeserver_host]
    self.autoguider_port = 54040
    self.autoguider_available = False
    self.autoguider_connected = False
    self.autoguider_autoengage = False
    self.autoguider_guiding = False
    self.autoguider_status = 'idle'
    self.autoguider_img_count = 0
    self.autoguider_gamma = 1.0
    self.autoguider_bp = 0
    self.autoguider_mag = 1
    self.autoguider_exposure = 1.0
    self.autoguider_interval = 1.0
    self.guide_correction_max_iters = 100
    self.guide_correction_iters = 0

    self.dec_axis_running = False
    self.dec_axis_goto = False
    self.ra_axis_running = False
    self.ra_axis_goto = False
    # high precision mode updates the pos_ra of the RA target on every iteration
    # low precision mode does not update the pos_ra of the RA target
    self.ra_axis_goto_precision = 'high'  # high, low
    self.ra_axis_tracking = False
    self.pec_tracking = True
    self.pec_tracking_status = 'ready'  # allowed values: 'ready' 'cancel'
    self.pec_rate = 0
    self.ra_pos_pre_index = 0
    self.ra_axis_homing = False
    self.ra_axis_darv_pause = False
    self.ra_axis_darv_east = False
    self.ra_axis_darv_west = False
    self.goto_target_running = False
    self.resume_mode = 'NONE'  # allowed functions:  NONE, GUIDE, GOTO
    self.meridian_mode = 1  # allowed values:  1 (for N_CW), -1 (for N_CCW)
    self.encoder_res_ra = 40000
    self.encoder_res_dec = 40000
    self.worm_gear_ra = 576
    self.worm_gear_dec = 450
    self.res_ra = self.worm_gear_ra*self.encoder_res_ra
    self.res_dec = self.worm_gear_dec*self.encoder_res_dec
    self.degree_counts_ra=int(self.res_ra/360)
    self.degree_counts_dec=int(self.res_dec/360)
    self.ra_counts_per_arcsec = self.res_ra/(360*3600.)
#    self.dst = (time.localtime()[8])*3600.0  # Correct for Daylight Savings Time
#    self.dst = 2*3600.0  # Correct for Daylight Savings Time
    self.dst = (2-time.localtime()[8])*3600.0  # Correct for Daylight Savings Time
    self.update_counter = 0
    self.t_offset = 0.0
    self.t_jitter = 0.0

    try:
      import gpsd
      gpsd.connect()
      lat, lon = gpsd.get_current().position()
      self.site_latitude = lat
      self.site_longitude = lon
      self.gpsd_connected = True
    except:
      self.site_latitude = 33.083
      self.site_longitude = -117.246
      self.gpsd_connected = False

    tropical_year = 365.242190402
    sidereal_year = 365.25636
#    self.sidereal_day = 86400*tropical_year/(1.0+tropical_year)
#    self.sidereal_day = 86400.0/1.002737909350795
#    self.sidereal_day = 86400.0/1.00273781191135448

    # Calculate Local Sidereal Time
    y,m,d = time.gmtime()[0:3]
    j0 = 367*y - int(7*(y+int(m+9)/12)/4) - int(3*(int((y+(m-9)/7)/100)+1)/4) + int(275*m/9) + d + 1721028.5
    T0 = (j0-2451545)/36525.
#    theta_G0 = (100.4606184 + 36000.77004*T0 + 0.000387933*T0*T0 - 2.583e-8*T0*T0*T0)%360.
    theta_G0 = (360/86400*(24110.5493771 + 8640184.794536*T0 + 0.0931118*T0**2 - 0.0000062*T0**3 + 0.0000013*T0**4))%360
    d_frac = (time.time() - (time.mktime((y, m, d, 0, 0, 0, 0, 0, 0))-time.timezone))/86400.
#    theta_G = theta_G0 + 360.98564724*d_frac
    theta_G = theta_G0 + 360.9856122880876*d_frac
    theta = (theta_G + self.site_longitude)%360
    self.local_meridian = theta

    jd = j0 + d_frac

    julian_year = 365.25*86400
    j2000 = time.mktime((2000, 1, 1, 12, 0, 0, 0, 0, 0)) - time.timezone
    fc = (time.time() - j2000)/(julian_year*100)
    self.sidereal_day = 86400.0/(1.002737909350795 + 5.9006e-11*fc - 5.9e-15*fc**2)
#    rate_correction = 46/60.  # Adjust sidereal rate, perhaps due to servo clock?
    rate_correction = 0.  # Don't apply any rate correction
    self.sidereal_rate = (self.res_ra/self.sidereal_day) - rate_correction

#    orbit_frac = (1 + ((time.time() + time.timezone - time.mktime((time.localtime()[0], 1, 1, 0, 0, 0, 0, 0, 0)))/86400))/tropical_year
    n_orbits = time.time()/(86400*sidereal_year)
    orbit_frac = n_orbits - math.trunc(n_orbits)

    d = (1 + ((time.time() - time.mktime((2018, 1, 1, 0, 0, 0, 0, 0, 0)))/86400))
    di = int(d)
    df = d - di
    dic = (self.res_ra*di/tropical_year)
    dfc = df*self.res_ra
#    horizon_pos = 29*64000 + self.res_ra - ((dic+dfc)%self.res_ra)
#    horizon_pos = (90.0 + self.site_longitude + 360*orbit_frac)*self.degree_counts_ra
#    horizon_pos = (99.25 + self.site_longitude + 360*orbit_frac)*self.degree_counts_ra
#    horizon_pos = (98.917 + self.site_longitude + 360*orbit_frac)*self.degree_counts_ra

    # Not sure why we need to add 0.028398 to orbit_frac but it works!
    orbit_offset = 0.028389
#    orbit_offset = -0.0132
    horizon_pos = (90 + self.site_longitude + 360*(orbit_frac+orbit_offset))*self.degree_counts_ra
    self.offset_pos = (0 + self.site_longitude + 360*(orbit_frac+orbit_offset))*self.degree_counts_ra
#    self.pos_ra = horizon_pos  # set home RA angle at Western horizon
    self.pos_ra = horizon_pos-self.offset_pos  # set home RA angle at Western horizon
    self.horizon_pos = self.pos_ra
    self.meridian_pos = 2*self.pos_ra

    # New way to setup coordinate system, 03/17/2021
    self.horizon_pos = 5760000  # set  RA encoder position of Western horizon
    self.meridian_pos = 11520000 # set RA encoder position of Meridian
    self.pos_ra = self.horizon_pos  # set home RA angle at Western horizon
    self.offset_pos = 44000
#    local_sidereal_time = (Time((Time.now()),scale='utc')).sidereal_time('apparent',longitude=self.site_longitude,model='IAU2006A')

    #self.pos_ra = 213.5*self.degree_counts_ra  # set home RA angle at Western horizon
    #self.pos_ra = 90*self.degree_counts_ra  # home RA angle is 90 degrees
    #self.pos_ra = 180*self.degree_counts_ra  # home RA angle is 98.99 degrees

    self.pos_dec = 0*self.degree_counts_dec  # home Dec angle is 0 degrees
    self.dec_angle = self.get_dec_angle()
    self.ra_axis_start_pos = self.pos_ra

    sys.stderr.write('\n\nSidereal Tracking Rate set to: %.6g  counts per second\n\n' % (self.sidereal_rate))
    self.slew_rate_max = 2*self.res_ra/360.
    self.slew_rate = self.slew_rate_max
    self.slew_rate_find = self.slew_rate_max/10
    self.slew_rate_center = self.slew_rate_max/50
    self.slew_rate_correction = 1.5*self.sidereal_rate
    self.slew_rate_track = self.sidereal_rate

    self.timezone = time.timezone

    sys.stderr.write('\nStarting ScopeSever in %s mode\n' % (self.scope_mode))
    sys.stderr.write('    GPS Fix: %s\n' % (str(self.gpsd_connected)))
    sys.stderr.write('    Site location set to: Lat: %.9g   Lon: %.9g\n\n' % (self.site_latitude, self.site_longitude))

    sys.stderr.write('    HorizonPos: %d   OffsetPos: %d  Pos RA: %d\n\n' % (int(self.horizon_pos), int(self.offset_pos), int(self.pos_ra)))

    self.target_ra_pos = 0.0
    self.sidereal_time = SiderealTime(longitude=self.site_longitude)
    self.target_ra_time = '00:00:00'
    self.target_ra_time_array = [0,0,0.0]
    self.target_dec_pos = 0.0
    self.target_dec_angle = "+00*00:00"
#    self.target_epsilon_1 = 1.0/2.0
    self.target_epsilon_1 = 300.0
    self.target_epsilon_2 = 2.0

    self.dRA = 0
    self.adj_RA = 0
    self.dDEC = 0
    self.guide_correction_pending = False
 
    if self.scope_mode == 'REAL_SCOPE':
      nmc_net = nmccom.NmcNet()
      nmc_net.Initialize(['RA','Dec'],baudrate=230400)
      self.ra_mod = nmc_net.modules['RA']
      self.dec_mod = nmc_net.modules['Dec']

      signal.signal(signal.SIGINT, signal_handler)
      signal.signal(signal.SIGHUP, signal_handler)
      signal.signal(signal.SIGTERM, signal_handler)
      signal.signal(signal.SIGQUIT, signal_handler)

      self.ra_mod.verbosity = 1
      self.dec_mod.verbosity = 1
      self.servo_cycle_rate = 1953.125
      self.servo_sidereal_rate = int(round(self.sidereal_rate*0.000512*2**16))
      self.servo_homing_rate = int(round(10*self.servo_cycle_rate*0.000512*2**16))
      self.servo_ra_correction_rate = int(round(self.slew_rate_correction*0.000512*2**16))
      self.servo_ra_slew_rate = int(round(self.slew_rate*0.000512*2**16))
      self.servo_dec_slew_rate = int(round(self.slew_rate*0.000512*2**16))
      self.servo_dec_correction_rate = int(round(self.slew_rate_correction*0.000512*2**16))


      self.servo_accel = 4000  # 4000
      self.servo_correction_accel = 32000  # 4000
      self.motor_current_ra = 0
      self.motor_current_dec = 0
      self.pos_error_ra = 0
      self.pos_error_dec = 0

      self.Kp_f = 200
      self.Kd_f = 800
      self.Ki_f = 800

      self.Kp_d = 600
      self.Kd_d = 0
      self.Ki_d = 600

      self.Kp_s = 600
      self.Kd_s = 0
      self.Ki_s = 600

#      self.Kp_s = self.Kp_f
#      self.Kd_s = self.Kd_f
#      self.Ki_s = self.Ki_f

      self.IL = 256
      self.OL = 255
      self.CL = 253
      self.EL = 4000
      self.SR = 1
      self.DB = 0
      self.SM = 1
 
      self.ra_mod.ServoIOControl(output_mode=nmccom.PH3_MODE)
      self.dec_mod.ServoIOControl(output_mode=nmccom.PH3_MODE)
#      self.ra_mod.ServoSetGain(200, 800, 200, 100, 255, 253, 4000, 1, 0, 1)
#      self.dec_mod.ServoSetGain(200, 800, 200, 100, 255, 253, 4000, 1, 0, 1)
      self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
      self.dec_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
      self.ra_mod.ServoSetPos(int(self.pos_ra))
      self.dec_mod.ServoSetPos(int(self.pos_dec))

      self.status_bits_ra = nmccom.SEND_POS | nmccom.SEND_CUR_SENSE | nmccom.SEND_POS_ERR
      self.status_bits_dec = nmccom.SEND_POS | nmccom.SEND_CUR_SENSE | nmccom.SEND_POS_ERR

      self.pec_home_phase_shift = -50
#      self.pec_home_phase_shift = 5000
#      self.pec_home_phase_shift = 15000
#      self.pec_home_phase_shift = 10000
#      self.pec_home_phase_shift = 21000
      self.pec_phase_shift = self.pec_home_phase_shift

      self.servo_pec_max_drift_rate = (self.ra_counts_per_arcsec*0.000512*2**16)*self.pec_rate_func()
      tmp_servo_sidereal_rate = self.sidereal_rate*0.000512*2**16
      self.servo_pec_sidereal_rate = (tmp_servo_sidereal_rate + self.servo_pec_max_drift_rate).round().astype('uint32')
      
   
      '''
      self.pec_max_drift_rate = (20/75.)  # arcsecs per second
      self.servo_pec_max_drift_rate = int(round((20/75.)*self.ra_counts_per_arcsec*0.000512*2**16))
      self.servo_pec_sidereal_rate = self.servo_sidereal_rate + self.servo_pec_max_drift_rate*self.pec_rate_func()
      '''

      self.pos_ra = self.ra_mod.ServoGetPos()
      pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
      self.pec_rate = int(self.servo_pec_sidereal_rate[pec_idx])

      '''
      # Example nmccom commands
      # Servo On
      #self.ra_mod.ServoStopMotor()
      #self.dec_mod.ServoStopMotor()

      # Servo Off
      #self.ra_mod.ServoStopMotorOff()
      #self.dec_mod.ServoStopMotorOff()

      # Slew RA at fast rate
      self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
      self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, 15*self.degree_counts_ra, self.servo_fast_rate, 400, 0)
      self.ra_mod.NoOp()
      while not self.ra_mod.response[0] & nmccom.MOVE_DONE:
        if i%200 == 0:
          self.ra_mod.PrintFullStatusReport()
        self.ra_mod.NoOp()
        i+=1


      # Slew RA at sidereal rate
      # Servo On
      #self.ra_mod.ServoStopMotor()
      pos = self.ra_mod.ServoGetPos()
      self.ra_mod.ServoSetGain(self.Kp_s, self.Kd_s, self.Ki_s, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
      self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, pos + 360*self.degree_counts_ra, servo_sidereal_rate, 100, 0)
      self.ra_mod.NoOp()
      while not self.ra_mod.response[0] & nmccom.MOVE_DONE:
        if i%200 == 0:
          self.ra_mod.PrintFullStatusReport()
        self.ra_mod.NoOp()
        i+=1
      '''


  def get_status(self):
    status_dict = {}

    self.motion_q.put(('update_pos', None))
    status_dict['site_local_time'] = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    status_dict['site_utc_time'] = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')

    if not self.update_counter%10:
      try:
        self.t_offset, self.t_jitter = [ float(item) for item in subprocess.check_output(['ntpq -p'],shell=True,stderr=subprocess.STDOUT).decode('utf-8').splitlines()[2].split()[8:] ]
      except:
        self.t_offset = 0.0
        self.t_jitter = 0.0

    # Try to reconnect to autoguider
    if self.autoguider_available and not self.autoguider_connected:
      self.autoguider_connect()

    if not self.update_counter%1:
      if self.autoguider_connected:
        self.get_autoguider_status()

    self.update_counter += 1

    status_dict['t_offset'] = self.t_offset
    status_dict['t_jitter'] = self.t_jitter

    status_dict['gps_status'] = self.gpsd_connected
    status_dict['site_latitude'] = self.site_latitude
    status_dict['site_longitude'] = self.site_longitude

    if self.meridian_mode == 1:
      status_dict['meridian_mode'] = 'N_CW'
    else:
      status_dict['meridian_mode'] = 'N_CCW'
    status_dict['dec_angle'] = self.get_dec_angle()
    status_dict['dec_pos'] = self.pos_dec
    status_dict['ra_time'] = self.get_ra_time()
    status_dict['ra_pos'] = self.pos_ra

    status_dict['target_dec_angle'] = self.target_dec_angle
    status_dict['target_dec_pos'] = self.target_dec_pos
    status_dict['target_ra_time'] = self.target_ra_time
    status_dict['target_ra_pos'] = self.ra_time_array_to_pos(self.target_ra_time_array)
    tmp_pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
    tmp_pec_rate = int(self.servo_pec_sidereal_rate[tmp_pec_idx])
    status_dict['pec_idx'] = tmp_pec_idx
    status_dict['pec_rate'] = tmp_pec_rate
    status_dict['pec_phase_shift'] = self.pec_phase_shift
    if self.pec_tracking:
      status_dict['pec_tracking'] = 'on'
    else:
      status_dict['pec_tracking'] = 'off'
    status_dict['motor_current_ra'] = self.motor_current_ra
    status_dict['motor_current_dec'] = self.motor_current_dec
    status_dict['pos_error_ra'] = self.pos_error_ra
    status_dict['pos_error_dec'] = self.pos_error_dec

    if self.autoguider_available:
      if self.autoguider_connected:
        tmpstr = 'connected'
      else:
        tmpstr = 'disconnected'
    else:
        tmpstr = 'unavailable'

#    if self.autoguider_autoengage:
#      tmpstr += ' autoengage'
#    else:
#      tmpstr += ' no-autoeng'

    status_dict['autoguider_connected'] = tmpstr
    status_dict['autoguider_status'] = '%s %d' % (self.autoguider_status, self.autoguider_img_count)
    status_dict['gamma_val_str'] = '%g' % (self.autoguider_gamma)
    status_dict['bp_val_str'] = '%g' % (self.autoguider_bp)
    status_dict['mag_val_str'] = '%g' % (self.autoguider_mag)
    status_dict['exposure_val_str'] = '%g' % (self.autoguider_exposure)
    status_dict['autoguider_interval_val_str'] = '%g' % (self.autoguider_interval)

    return status_dict


  def get_small_status(self):
    status_dict = {}
    status_dict['ra_pos'] = self.pos_ra
    status_dict['dec_pos'] = self.pos_dec
    status_dict['pec_idx'] = (self.pos_ra+self.pec_phase_shift)%40000
    status_dict['meridian_mode'] = self.meridian_mode

    return status_dict


  def set_meridian_mode(self, mode):
    if (mode != self.meridian_mode):
      self.meridian_mode = mode
      self.target_dec_pos = self.dec_angle_to_pos(self.target_dec_angle)
      self.dec_mod.ServoSetPos(int(self.pos_dec))
      self.pos_dec = self.dec_mod.ServoGetPos()
      self.dec_angle = self.get_dec_angle()
    self.meridian_mode = mode


  def meridian_flip(self):
    self.meridian_mode = -self.meridian_mode
#    mode = -self.meridian_mode
#    self.set_meridian_mode(mode)


  # Periodic Error Correction Rate Function
  def pec_rate_func(self):
    pec_rate_data = np.fromfile('/home/pi/src/scopeserver-git/scopeserver/server/pec_rate_real_sum_v2_v3.dat', sep=' ').reshape(-1,2)
    pec_rate = 0.0 + pec_rate_data[:,1]

    return pec_rate


  # Periodic Error Correction Rate Function
  def pec_rate_func_2(self,curv=0.01,peak_error=10.0,pec_period=150.0,n_pnts=40000):

    # Set t values for one min-to-max-to_min cycle 
    t = np.linspace(-0.25, 0.75, num=n_pnts)

    # Smooth Triangle Function, curv controls the sharpness curvature
    curv = 0.01
    y = 1 - 2*np.arccos((1-curv)*np.sin(2*np.pi*t))/np.pi

    # Normalize and Scale y
    y = peak_error*y/y.max()

    # Offset and Scale t
    t = pec_period*(t+0.25)

    # Cubic spline of y(t)
    spl= UnivariateSpline(t, y, k=3, s=0)

    # First derivative of y(t)
    d_spl = spl.derivative()

    pec_rate = d_spl(t)

    return pec_rate


  # Get declination angle position of scope
  def get_dec_angle(self):
    pos = self.meridian_mode*(self.res_dec - self.pos_dec)%self.res_dec
    dec_angle = 360.0*pos/self.res_dec
    dec_dd = int(dec_angle)
    dec_rem = 60*abs(dec_angle - dec_dd)
    dec_mm = int(dec_rem)
    dec_ss = int(60*abs(dec_rem - dec_mm))
    return ("%+.2d*%.2d:%.2d" % (dec_dd, dec_mm, dec_ss))
    

  # Convert Dec angle string to shaft pos
  def dec_angle_to_pos(self,dec_angle_str):
    dd = (dec_angle_str.split('*')[0])
    dd_sign = float(dec_angle_str[0] + '1')
    mm,ss = (dec_angle_str.split('*')[1]).split(':')
    dd = float(dd)
    mm = float(mm)
    ss = float(ss)
    dec_angle = (dd + (dd_sign*(mm/60.0 + ss/3600.0)))%360.0
    pos = self.meridian_mode*(self.res_dec - (self.res_dec*dec_angle/360.0))%self.res_dec
    if pos > (self.res_dec/2):
       pos = pos - self.res_dec
    if self.verbose:
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
  # New Method using SiderealTime()
  def get_ra_time(self):
    # In a moving reference frame with rotation of Earth:
    # Compute angle of RA position from the Western Horizon
    angle_ra = 360.*(self.pos_ra - self.meridian_pos)/self.res_ra
    
    # Compute sidereal time position of RA axis
    self.sidereal_time.update_sidereal_time(longitude=self.site_longitude+angle_ra)

    ra_time_str = self.sidereal_time.ra_time_str
    return (ra_time_str)


  '''
  # Get RA time position of scope
  # Astropy Method
  def get_ra_time(self):
    # In a moving reference frame with rotation of Earth:
    # Compute angle of RA position from the Western Horizon
#    angle_ra = 360.0*(self.pos_ra - self.horizon_pos - self.offset_pos)/self.res_ra
    angle_ra = 360.*(self.pos_ra - self.meridian_pos)/self.res_ra
    
    # Compute sidereal time position of RA axis
#    ra_time_ha = (Time((Time.now()),scale='utc')).sidereal_time('apparent',longitude=self.site_longitude-90.+angle_ra,model='IAU2006A')
    ra_time_ha = (Time((Time.now()),scale='utc')).sidereal_time('apparent',longitude=self.site_longitude+angle_ra,model='IAU2006A')

    ra_time_str = ra_time_ha.to_string(sep=':',pad=True,precision=0)
    return (ra_time_str)
  '''


  '''
  # Get RA time position of scope
  def get_ra_time(self):
    # In a moving reference frame with rotation of Earth:
    pos_ra = self.pos_ra + self.offset_pos
    ra_time = time.time() + self.dst + 86400.0*pos_ra/self.res_ra
    ra_time_str = time.ctime(ra_time).split()[3]
    return (ra_time_str)
#    return ('%.2d:%.2d.0' % (ra_hh, ra_mm))
  '''


  def set_target_ra_time_array(self,ra_time_str):
    ra_time = ra_time_str.split(':')
    self.target_ra_time_array = []
    self.target_ra_time_array.append(int(ra_time[0]))
    self.target_ra_time_array.append(int(ra_time[1]))
    self.target_ra_time_array.append(float(ra_time[2]))
    pos = self.ra_time_array_to_pos(self.target_ra_time_array)
    if self.verbose:
      sys.stderr.write('Target RA: %s  pos %.9g\r\n' % (ra_time_str, pos))
    return


  def adj_target_ra_time_array(self, adj_RA):
    t = (3600*self.target_ra_time_array[0] + 60*self.target_ra_time_array[1] + self.target_ra_time_array[2] + adj_RA)%86400.0
    hh_f = 24.0*t/86400.0
    hh = int(hh_f)
    rem = 60*abs(hh_f - hh)
    mm = int(rem)
    ss = 60*abs(rem - mm)
    self.target_ra_time_array = [hh,mm,ss]
    pos = self.ra_time_array_to_pos(self.target_ra_time_array)
    if self.verbose:
      sys.stderr.write('Adjusted Target RA: %s  pos %.9g\r\n' % (str(self.target_ra_time_array), pos))
    return


  # Convert RA time array to shaft pos
  # New Method using SiderealTime()
  def ra_time_array_to_pos(self,ra_time_array):
    # In a moving reference frame with rotation of Earth:
    ra_time_angle = (3600*ra_time_array[0] + 60*ra_time_array[1] + ra_time_array[2])*(360./86400.)

    self.sidereal_time.update_sidereal_time(longitude=self.site_longitude)

    delta_angle = ra_time_angle - self.sidereal_time.ERA
    delta_pos = delta_angle*self.res_ra/360.0

#    pos = int((self.horizon_pos + delta_angle*self.degree_counts_ra + self.offset_pos)%self.res_ra)
    pos = int((self.meridian_pos + delta_pos)%self.res_ra)

#    sys.stderr.write('Target ra_time: %02d:%02d:%02d  pos %.9g\r\n' % (ra_time_array[0], ra_time_array[1], ra_time_array[2], pos))
    return (pos)


  '''
  # Convert RA time array to shaft pos
  # Astropy Method
  def ra_time_array_to_pos(self,ra_time_array):
    # In a moving reference frame with rotation of Earth:
    ra_time_angle = (3600*ra_time_array[0] + 60*ra_time_array[1] + ra_time_array[2])*(360./86400.)
#    horizon_time_angle = (Time((Time.now()),scale='utc')).sidereal_time('apparent',longitude=self.site_longitude-90.,model='IAU2006A').degree
    meridian_time_angle = (Time((Time.now()),scale='utc')).sidereal_time('apparent',longitude=self.site_longitude,model='IAU2006A').degree

    delta_angle = ra_time_angle - meridian_time_angle
    delta_pos = delta_angle*self.res_ra/360.0

#    pos = int((self.horizon_pos + delta_angle*self.degree_counts_ra + self.offset_pos)%self.res_ra)
    pos = int((self.meridian_pos + delta_pos)%self.res_ra)

#    sys.stderr.write('Target ra_time: %02d:%02d:%02d  pos %.9g\r\n' % (ra_time_array[0], ra_time_array[1], ra_time_array[2], pos))
    return (pos)
  '''


  '''
  # Convert RA time array to shaft pos
  def ra_time_array_to_pos(self,ra_time_array):
    # In a moving reference frame with rotation of Earth:
    tt = list(time.localtime(time.time()))
#    tt[3:6] = ra_time_array[:]
    tt[3:5] = ra_time_array[:-1]
    tt[5] = int(ra_time_array[2])
    ra_time = time.mktime(tuple(tt)) + math.modf(ra_time_array[2])[0]
#    pos = ((ra_time - time.time() - self.dst)*self.res_ra/86400.0)%self.res_ra
    pos = ((ra_time - time.time() - self.dst)*self.res_ra/86400.0)
    pos = (pos-self.offset_pos)%self.res_ra
#    sys.stderr.write('target ra_time: %02d:%02d:%02d  pos %.9g\r\n' % (ra_time_array[0], ra_time_array[1], ra_time_array[2], pos))
    return (pos)
  '''


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


  # Start thread to get ntpq data
  def ntpq_start(self):
    self.thread_get_ntpq_data = threading.Thread(target=self.get_ntpq_data)
    self.thread_get_ntpq_data.setDaemon(1)
    self.thread_get_ntpq_data.start()


  def get_ntpq_data(self):
    sys.stderr.write('get_ntpq_data: thread starting...\r\n')
    while True:
      time.sleep(10)
      self.t_offset, self.t_jitter = subprocess.check_output(['ntpq -p'],shell=True).decode('utf-8').splitlines()[3].split()[8:]


###########################
# Motion Control Methods
###########################

  def autoguider_latency_test(self):
    HOST = self.autoguider_host
    PORT = self.autoguider_port

    cmd_template = '{get_autoguider_time %s}'

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
          cmd = cmd_template % (str(t0))
          sock.sendall(bytes(cmd, 'utf-8'))

          # Receive response from the server and finish
          response = str(sock.recv(1024), 'utf-8').split()
          t2 = time.time_ns()
        except:
          response = [] 
        finally:
          sock.close()

      if len(response):
        t_ag = int(response[1])
        dt = t2 - t0
        t_diff = (t_ag - dt/2) - t0
        dt_array = np.append(dt_array,dt)
        t_diff_array = np.append(t_diff_array,t_diff)
      else:
        sys.stderr.write('\r\nunknown response from autoguider %s\r\n' % (response))
      
    dt = dt_array*1e-9
    t_diff = t_diff_array*1e-9
    if self.verbose:
      sys.stderr.write('\r\ndt = %.4g (min:%.4g max:%.4g stddev:+-%.4g)\r\n' % (dt.mean(),dt.min(),dt.max(),dt.std()))
      sys.stderr.write('t_diff = %.4g (min:%.4g max:%.4g stddev:+-%.4g)\r\n' % (t_diff.mean(),t_diff.min(),t_diff.max(),t_diff.std()))


  # Start thread for motion control
  def motion_control_start(self):
    self.thread_motion_control = threading.Thread(target=self.motion_control)
    self.thread_motion_control.setDaemon(1)
    self.thread_motion_control.start()


  def motion_control(self):
    sys.stderr.write('\r\nmotion_control: thread starting...\r\n')
    while True:
      if self.motion_running:
        # if in motion get motion command but do not wait if queue is empty
        try:
          motion_cmd = self.motion_q.get_nowait()
        except queue.Empty:
          motion_cmd = ('motion_continue', None)
      else:
        # if not in motion wait to receive motion command
#        sys.stderr.write('motion_control: idle, waiting for command...\r\n')
        motion_cmd = self.motion_q.get()

      cmd = motion_cmd[0]
      cmd_arg = motion_cmd[1]

      if cmd == 'ag_connect':
        self.autoguider_connect()

      elif cmd == 'autoguider_latency_test':
        self.autoguider_latency_test()

      elif cmd == 'autoguider_guide_start':
        self.autoguider_guiding = True
        self.autoguider_guide_start()

      elif cmd == 'autoguider_guide_stop':
        self.autoguider_guiding = False 
        self.autoguider_guide_stop()
        # Cancel pending guide correction
        self.guide_correction_pending = False
        self.dRA = 0
        self.adj_RA = 0.0
        self.dDEC = 0

      elif cmd == 'reboot_autoguider':
        self.reboot_autoguider()

      elif cmd == 'shutdown_autoguider':
        self.shutdown_autoguider()

      elif cmd == 'set_ra_index_pos_step_1':
        self.ra_axis_homing = True
        self.ra_axis_running = True
        # Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        # SetHoming Mode
        self.ra_mod.ServoSetHoming()
        # Slew RA forward one revolution at homing rate
        self.pos_ra_pre_index = self.ra_mod.ServoGetPos()
        self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, self.pos_ra-40000, self.servo_homing_rate, 400, 0)

      elif cmd == 'set_ra_index_pos_step_2':
        self.ra_axis_running = True
        # Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        # Slew RA to return to original pre-index position
        self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, self.pos_ra_pre_index, self.servo_homing_rate, 400, 0)

      elif cmd == 'toggle_pec':
        self.pec_tracking = not self.pec_tracking
        if self.pec_tracking:
          self.pos_ra = self.ra_mod.ServoGetPos()
          pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
          self.pec_rate = int(self.servo_pec_sidereal_rate[pec_idx])
        else:
          self.pec_rate = self.servo_sidereal_rate
        if self.ra_axis_tracking:
          if self.pec_tracking:
            self.pos_ra = self.ra_mod.ServoGetPos()
            pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
            self.pec_rate = int(self.servo_pec_sidereal_rate[pec_idx])
#            sys.stderr.write('Transition to PEC Tracking: %d\r\n' % (self.pec_rate))
            self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -360*self.degree_counts_ra, self.pec_rate, self.servo_accel, 0)
          else:
            self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -360*self.degree_counts_ra, self.servo_sidereal_rate, self.servo_accel, 0)

      elif cmd == 'jog_pos':
        jog_ra = cmd_arg[0]
        jog_dec = cmd_arg[1]
        if self.ra_axis_tracking:
          # Servo On
          self.ra_mod.ClearBits()
          self.ra_mod.ServoStopMotor()
          self.pos_ra = self.ra_mod.ServoGetPos()
          self.dec_mod.ClearBits()
          self.dec_mod.ServoStopMotor()
          self.pos_dec = self.dec_mod.ServoGetPos()

          self.ra_mod.ServoSetGain(self.Kp_s, self.Kd_s, self.Ki_s, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
          self.dec_mod.ServoSetGain(self.Kp_s, self.Kd_s, self.Ki_s, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)

          jog_ra_done = True
          jog_dec_done = True

          if jog_ra != 0:
            # Jog RA
            self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, jog_ra + self.pos_ra, 10*self.servo_sidereal_rate, self.servo_accel, 0)
            jog_ra_done = False

          if jog_dec != 0:
            # Jog Dec
            self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, jog_dec + self.pos_dec, 10*self.servo_sidereal_rate, self.servo_accel, 0)
            jog_ra_done = False

          while (not jog_ra_done) or (not jog_dec_done) :
            self.ra_mod.ReadStatus(self.status_bits_dec)
            self.dec_mod.ReadStatus(self.status_bits_dec)
            jog_ra_done = self.ra_mod.response[0] & nmccom.MOVE_DONE
            jog_dec_done = self.dec_mod.response[0] & nmccom.MOVE_DONE

            if jog_ra_done:
              pass

            if jog_dec_done:
              # Servo Off
              self.dec_mod.ServoStopMotorOff()
              self.dec_axis_running = False
              self.dec_mod.ReadStatus(self.status_bits_dec)
              self.pos_dec = self.dec_mod.status_dict['pos']
              self.motor_current_dec = self.dec_mod.status_dict['cur_sense']
              self.pos_error_dec = self.dec_mod.status_dict['pos_error']

          # Resume tracking
          self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -360*self.degree_counts_ra, self.servo_sidereal_rate, self.servo_accel, 0)

      elif cmd == 'ra_slew_start':
        if self.verbose:
          sys.stderr.write('motion_control: RA slew start\r\n')
        # RA Slew at slew rate
        self.ra_axis_running = True
        # RA Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.ra_axis_start_pos = self.pos_ra
        self.ra_axis_start_time = time.time()
        servo_slew_rate = abs(self.servo_ra_slew_rate - cmd_arg*self.servo_sidereal_rate)
        self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, cmd_arg*360*self.degree_counts_ra, servo_slew_rate, self.servo_accel, 0)

      elif cmd == 'ra_slew_stop':
        if self.verbose:
          sys.stderr.write('motion_control: RA slew stop\r\n')
        # RA Smooth Stop
        if self.ra_axis_running | self.ra_axis_tracking:
          self.pec_tracking_status = 'cancel'
          self.ra_mod.ServoStopSmooth()
        else:
          self.ra_mod.ServoStopMotorOff()
        if self.autoguider_connected:
          self.motion_q.put(('autoguider_guide_stop', None))

      elif cmd == 'ra_track_start':
        if self.verbose:
          sys.stderr.write('motion_control: RA track start\r\n')
        # Slew RA at sidereal rate
        self.ra_axis_tracking = True
        # Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.ra_axis_track_start_pos = self.pos_ra
        self.ra_mod.ServoSetGain(self.Kp_s, self.Kd_s, self.Ki_s, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        if self.pec_tracking:
          # Track with PEC 
          pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
          self.pec_rate = int(self.servo_pec_sidereal_rate[pec_idx])
#          sys.stderr.write('Begin PEC Tracking: %d\r\n' % (self.pec_rate))
          self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -360*self.degree_counts_ra, self.pec_rate, self.servo_accel, 0)
        else:
          self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -360*self.degree_counts_ra, self.servo_sidereal_rate, self.servo_accel, 0)

      elif cmd == 'ra_track_stop':
        if self.verbose:
          sys.stderr.write('motion_control: RA track stop\r\n')
        # RA Smooth Stop
        if self.ra_axis_running | self.ra_axis_tracking:
          self.pec_tracking_status = 'cancel'
          self.ra_mod.ServoStopSmooth()
        else:
          self.ra_mod.ServoStopMotorOff()
        if self.autoguider_connected:
          self.motion_q.put(('autoguider_guide_stop', None))

      elif cmd == 'ra_darv_pause':
        if self.verbose:
          sys.stderr.write('motion_control: RA DARV Pause start\r\n')
        # Slew RA at sidereal rate
        self.ra_axis_running = True
        self.ra_axis_darv_pause = True
        # Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        darv_time = 10.0
        darv_distance = darv_time*self.sidereal_rate
        darv_rate = self.sidereal_rate
        servo_darv_rate = int(round(darv_rate*0.000512*2**16))
        self.ra_axis_darv_start_pos = self.pos_ra
        self.ra_axis_darv_end_pos = int(self.pos_ra-darv_distance)
        self.ra_mod.ServoSetGain(self.Kp_d, self.Kd_d, self.Ki_d, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, self.ra_axis_darv_end_pos, servo_darv_rate, self.servo_accel, 0)

      elif cmd == 'ra_darv_east':
        if self.verbose:
          sys.stderr.write('motion_control: RA DARV East start\r\n')
        # Slew RA at sidereal rate
        self.ra_axis_running = True
        self.ra_axis_darv_east = True
        # Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        darv_time = 180.0
        darv_degrees = 1.5
        darv_distance = darv_degrees*self.degree_counts_ra
        darv_distance = darv_distance + darv_time*self.sidereal_rate
        darv_rate = darv_distance/darv_time
        servo_darv_rate = int(round(darv_rate*0.000512*2**16))
        self.ra_axis_darv_start_pos = self.pos_ra
        self.ra_axis_darv_end_pos = int(self.pos_ra-darv_distance)
        self.ra_mod.ServoSetGain(self.Kp_d, self.Kd_d, self.Ki_d, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, self.ra_axis_darv_end_pos, servo_darv_rate, self.servo_accel, 0)

      elif cmd == 'ra_darv_west':
        if self.verbose:
          sys.stderr.write('motion_control: RA DARV West start\r\n')
        # Slew RA at sidereal rate
        self.ra_axis_running = True
        self.ra_axis_darv_west = True
        # Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        darv_time = 180.0
        darv_degrees = 1.5
        darv_distance = darv_degrees*self.degree_counts_ra
        darv_distance = darv_distance - darv_time*self.sidereal_rate
        darv_rate = darv_distance/darv_time
        servo_darv_rate = int(round(darv_rate*0.000512*2**16))
        self.ra_axis_darv_start_pos = self.pos_ra
        self.ra_axis_darv_end_pos = int(self.pos_ra+darv_distance)
        self.ra_mod.ServoSetGain(self.Kp_d, self.Kd_d, self.Ki_d, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, self.ra_axis_darv_end_pos, servo_darv_rate, self.servo_accel, 0)

      elif cmd == 'dec_slew_start':
        if self.verbose:
          sys.stderr.write('motion_control: Dec slew start\r\n')
        # Dec Slew at slew rate
        self.dec_axis_running = True
        # Dec Servo On
        self.dec_mod.ClearBits()
        self.dec_mod.ServoStopMotor()
        self.pos_dec = self.dec_mod.ServoGetPos()
        self.dec_axis_start_pos = self.pos_dec
        self.dec_axis_start_time = time.time()
        self.dec_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, cmd_arg*360*self.degree_counts_dec, self.servo_dec_slew_rate, self.servo_accel, 0)

      elif cmd == 'dec_slew_stop':
        if self.verbose:
          sys.stderr.write('motion_control: Dec slew stop\r\n')
        # Dec Smooth Stop
        if self.dec_axis_running:
          self.dec_mod.ServoStopSmooth()
        else:
          self.dec_mod.ServoStopMotorOff()

      elif cmd == 'goto_target_start':
        if self.verbose:
          sys.stderr.write('motion_control: GOTO target start\r\n')
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
          self.target_dec_pos = self.target_dec_pos - self.res_dec

        target_distance = math.sqrt(ra_distance**2 + dec_distance**2)
        ra_speed = abs(ra_distance/target_distance)
        dec_speed = abs(dec_distance/target_distance)
        if cmd_arg == 'center_rate':
          self.servo_ra_goto_rate = int(self.slew_rate_find*0.000512*2**16)
          self.servo_dec_goto_rate = int(self.slew_rate_find*0.000512*2**16)
          accel = self.servo_accel
        elif cmd_arg == 'correction_rate':
          self.servo_ra_goto_rate = int(self.slew_rate_correction*0.000512*2**16)
          self.servo_dec_goto_rate = int(self.slew_rate_correction*0.000512*2**16)
          accel = self.servo_correction_accel
        else:
          self.servo_ra_goto_rate = int(ra_speed*self.slew_rate*0.000512*2**16)
          self.servo_dec_goto_rate = int(dec_speed*self.slew_rate*0.000512*2**16)
          accel = self.servo_accel

        # RA GOTO Target at goto rate
        self.ra_axis_running = True
        self.ra_axis_goto = True
        # RA Servo On
        self.ra_mod.ClearBits()
        self.ra_mod.ServoStopMotor()
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.ra_axis_start_pos = self.pos_ra
        self.ra_axis_start_time = time.time()

        # Dec GOTO Target at goto rate
        self.dec_axis_running = True
        self.dec_axis_goto = True
        # Dec Servo On
        self.dec_mod.ClearBits()
        self.dec_mod.ServoStopMotor()
        self.pos_dec = self.dec_mod.ServoGetPos()
        self.dec_axis_start_pos = self.pos_dec
        self.dec_axis_start_time = time.time()

        self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
        self.dec_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)

        self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), self.servo_ra_goto_rate, accel, 0)
        self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_dec_pos), self.servo_dec_goto_rate, accel, 0)

      elif cmd == 'goto_target_stop':
        if self.verbose:
          sys.stderr.write('motion_control: GOTO target stop\r\n')

      elif cmd == 'align_to_target':
        if not self.motion_running:
          new_pos_ra = self.ra_time_array_to_pos(self.target_ra_time_array)
#          if (pos_ra <= self.meridian_pos):
#            self.set_meridian_mode(1)
#          else:
#            self.set_meridian_mode(-1)

          # Get current pec index
          self.pos_ra = self.ra_mod.ServoGetPos()
          pec_idx = (self.pos_ra+self.pec_phase_shift)%40000

          # Set new pec phase shift
          new_pec_phase_shift = (pec_idx - new_pos_ra)%40000
          self.pec_phase_shift = new_pec_phase_shift

          new_pos_dec = self.target_dec_pos
          self.ra_mod.ServoSetPos(int(new_pos_ra))
          self.dec_mod.ServoSetPos(int(new_pos_dec))
          self.pos_ra = self.ra_mod.ServoGetPos()
          self.pos_dec = self.dec_mod.ServoGetPos()
          if self.verbose:
            sys.stderr.write('Aligned servo to target at:  pos_ra: %d   pos_dec: %d\r\n' % (self.pos_ra, self.pos_dec))
          if cmd_arg:
            self.motion_q.put(('ra_track_start', None))

      elif (cmd == 'motion_continue') | (cmd == 'update_pos'):
        self.pos_ra = self.ra_mod.ServoGetPos()
        self.pos_dec = self.dec_mod.ServoGetPos()
        if cmd_arg:
          if self.verbose:
            sys.stderr.write('  Current Pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))


#  Monitor and respond to motion dynamics:  

      if self.ra_axis_goto:
        # Make sure pos_ra is within target_epsilon_2 of the target
        # Update RA GOTO Target which has moved during the GOTO slewing
        if self.ra_mod.module_error:
          if self.verbose:
            sys.stderr.write('RA Drive Controller Error!!!\r\n')
          # Servo Off
          self.ra_mod.ServoStopMotorOff()
          self.ra_mod.ClearBits()
          self.ra_axis_running = False
          self.ra_axis_tracking = False
          self.ra_axis_goto = False
          self.ra_mod.ReadStatus(self.status_bits_ra)
          self.pos_ra = self.ra_mod.status_dict['pos']
          self.motor_current_ra = self.ra_mod.status_dict['cur_sense']
          self.pos_error_ra = self.ra_mod.status_dict['pos_error']
        else:
          # Update target_ra_pos for high precision goto
          self.pos_ra = self.ra_mod.ServoGetPos()
          if self.ra_axis_goto_precision == 'high':
            self.target_ra_pos = self.ra_time_array_to_pos(self.target_ra_time_array)
            target_ra_delta = (self.pos_ra - self.target_ra_pos)
            # sys.stderr.write('RA GOTO target_ra_delta = %d\r\n' % (target_ra_delta))
            target_ra_delta = abs(self.pos_ra - self.target_ra_pos)
            # Continue GOTO until we're within target_epsilon_2 
            if target_ra_delta > self.target_epsilon_2:
              if target_ra_delta > self.target_epsilon_1:
                if self.verbose:
                  sys.stderr.write('Slewing to within target_epsilon_1: %d\r\n' % (target_ra_delta))
                # Move at current slew rate until we're within target_epsilon_1
                self.ra_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
                # self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), 0)
                self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), self.servo_ra_goto_rate, self.servo_accel, 0)
#              else:
              else:
                if self.verbose:
                  sys.stderr.write('Slewing RA to within target_epsilon_2: %d\r\n' % (target_ra_delta))
                if self.autoguider_guiding:
                  # Move at correction slew rate until we're within target_epsilon_2
                  # self.ra_mod.ServoSetGain(self.Kp_s, self.Kd_s, self.Ki_s, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
                  self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), self.servo_ra_correction_rate, self.servo_correction_accel, 0)
                  # self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), 0)
                else:
                  self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_ra_pos), self.servo_ra_goto_rate, self.servo_accel, 0)
            else:
              # We're now within epsilon_2 so we can stop the correction
              # and resume tracking
              if self.verbose:
                sys.stderr.write('Within epsilon_2. Resume tracking...\r\n')
              # RA Servo Off
              self.ra_mod.ServoStopMotorOff()
              self.ra_axis_running = False
              self.ra_axis_tracking = False
              self.pec_tracking_status = 'ready'
              self.ra_axis_homing = False
              self.ra_axis_darv_pause = False
              self.ra_axis_darv_east = False
              self.ra_axis_darv_west = False
              self.ra_axis_goto = False
              self.ra_mod.ReadStatus(self.status_bits_ra)
              self.pos_ra = self.ra_mod.status_dict['pos']
              self.motor_current_ra = self.ra_mod.status_dict['cur_sense']
              self.pos_error_ra = self.ra_mod.status_dict['pos_error']
              self.guide_correction_iters = 0
              self.motion_q.put(('ra_track_start', None))

      if self.ra_axis_running | self.ra_axis_tracking:
        if self.ra_mod.module_error:
          if self.verbose:
            sys.stderr.write('RA Drive Controller Error!!!\r\n')
          # Servo Off
          self.ra_mod.ServoStopMotorOff()
          self.ra_mod.ClearBits()
          self.ra_axis_running = False
          self.ra_axis_tracking = False
          self.ra_axis_darv_pause = False
          self.ra_axis_darv_east = False
          self.ra_axis_darv_west = False
          self.ra_axis_goto = False
          self.ra_mod.ReadStatus(self.status_bits_ra)
          self.pos_ra = self.ra_mod.status_dict['pos']
          self.motor_current_ra = self.ra_mod.status_dict['cur_sense']
          self.pos_error_ra = self.ra_mod.status_dict['pos_error']
        else:
          self.ra_mod.ReadStatus(self.status_bits_ra)
          self.pos_ra = self.ra_mod.status_dict['pos']
          self.motor_current_ra = self.ra_mod.status_dict['cur_sense']
          self.pos_error_ra = self.ra_mod.status_dict['pos_error']
          if self.ra_mod.response[0] & nmccom.MOVE_DONE:
            if self.ra_axis_goto:
              self.motion_q.put(('ra_track_start', None))
            if self.ra_axis_darv_pause:
              self.motion_q.put(('ra_darv_east', None))
            if self.ra_axis_darv_east:
              self.motion_q.put(('ra_darv_west', None))
            if self.ra_axis_homing:
              self.pos_ra = self.ra_mod.ServoGetPos()

              new_pec_phase_shift = (self.pec_home_phase_shift - self.pos_ra)%40000
              self.pec_phase_shift = new_pec_phase_shift

              self.pos_ra = self.ra_mod.ServoGetPos()
              pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
              self.pec_rate = int(self.servo_pec_sidereal_rate[pec_idx])
              self.motion_q.put(('set_ra_index_pos_step_2', None))

#              self.ra_mod.ServoSetPos(self.horizon_pos)
#              self.pos_ra = self.ra_mod.ServoGetPos()
            # Servo Off
            self.ra_mod.ServoStopMotorOff()
            self.ra_axis_running = False
            self.ra_axis_tracking = False
            self.pec_tracking_status = 'ready'
            self.ra_axis_homing = False
            self.ra_axis_darv_pause = False
            self.ra_axis_darv_east = False
            self.ra_axis_darv_west = False
            self.ra_axis_goto = False
            self.ra_mod.ReadStatus(self.status_bits_ra)
            self.pos_ra = self.ra_mod.status_dict['pos']
            self.motor_current_ra = self.ra_mod.status_dict['cur_sense']
            self.pos_error_ra = self.ra_mod.status_dict['pos_error']
          else:
            if self.ra_axis_tracking & self.pec_tracking & (self.pec_tracking_status == 'ready'):
              # Track with PEC 
              self.pos_ra = self.ra_mod.ServoGetPos()
              pec_idx = (self.pos_ra+self.pec_phase_shift)%40000
              self.pec_rate = int(self.servo_pec_sidereal_rate[pec_idx])
#              sys.stderr.write('Update PEC Tracking: %d\r\n' % (self.pec_rate))
              self.ra_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, -360*self.degree_counts_ra, self.pec_rate, self.servo_accel, 0)

      if self.dec_axis_goto:
        # Make sure pos_dec is within target_epsilon_2 of the target
        if self.dec_mod.module_error:
          if self.verbose:
            sys.stderr.write('DEC Drive Controller Error!!!\r\n')
          # Servo Off
          self.dec_mod.ServoStopMotorOff()
          self.dec_mod.ClearBits()
          self.dec_axis_running = False
          self.dec_axis_goto = False
          self.dec_mod.ReadStatus(self.status_bits_ra)
          self.pos_dec = self.dec_mod.status_dict['pos']
          self.motor_current_dec = self.dec_mod.status_dict['cur_sense']
          self.pos_error_dec = self.dec_mod.status_dict['pos_error']
        else:
          self.pos_dec = self.dec_mod.ServoGetPos()
          target_dec_delta = (self.pos_dec - self.target_dec_pos)
          # sys.stderr.write('DEC GOTO target_dec_delta = %d\r\n' % (target_dec_delta))
          target_dec_delta = abs(self.pos_dec - self.target_dec_pos)
          # Continue GOTO until we're within target_epsilon_2 
          if target_dec_delta > self.target_epsilon_2:
            if target_dec_delta > self.target_epsilon_1:
              # Move at current slew rate until we're within target_epsilon_1
              self.dec_mod.ServoSetGain(self.Kp_f, self.Kd_f, self.Ki_f, self.IL, self.OL, self.CL, self.EL, self.SR, self.DB, self.SM)
              self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_dec_pos), self.servo_dec_goto_rate, self.servo_accel, 0)
              # sys.stderr.write('Slewing to within target_epsilon_1: %d\r\n' % (target_dec_delta))
            else:
              if self.verbose:
                sys.stderr.write('Slewing DEC to within target_epsilon_2: %d\r\n' % (target_dec_delta))
              # Move at correction slew rate until we're within target_epsilon_2
              self.dec_mod.ServoLoadTraj(nmccom.LOAD_POS | nmccom.LOAD_VEL | nmccom.LOAD_ACC | nmccom.ENABLE_SERVO | nmccom.START_NOW, int(self.target_dec_pos), self.servo_dec_correction_rate, self.servo_correction_accel, 0)

      if self.dec_axis_running:
        if self.dec_mod.module_error:
          if self.verbose:
            sys.stderr.write('Dec Drive Controller Error!!!\r\n')
          # Servo Off
          self.dec_mod.ServoStopMotorOff()
          self.dec_mod.ClearBits()
          self.dec_axis_running = False
          self.dec_mod.ReadStatus(self.status_bits_dec)
          self.pos_dec = self.dec_mod.status_dict['pos']
          self.motor_current_dec = self.dec_mod.status_dict['cur_sense']
          self.pos_error_dec = self.dec_mod.status_dict['pos_error']
        else:
          self.dec_mod.ReadStatus(self.status_bits_dec)
          self.pos_dec = self.dec_mod.status_dict['pos']
          self.motor_current_dec = self.dec_mod.status_dict['cur_sense']
          self.pos_error_dec = self.dec_mod.status_dict['pos_error']
          if self.dec_mod.response[0] & nmccom.MOVE_DONE:
            # Servo Off
            self.dec_mod.ServoStopMotorOff()
            self.dec_axis_running = False
            self.dec_axis_goto = False
            self.dec_mod.ReadStatus(self.status_bits_dec)
            self.pos_dec = self.dec_mod.status_dict['pos']
            self.motor_current_dec = self.dec_mod.status_dict['cur_sense']
            self.pos_error_dec = self.dec_mod.status_dict['pos_error']


      # Redundant method to terminate guide correction:
      if self.guide_correction_pending and (self.ra_axis_goto or self.dec_axis_goto):
        # if a correction is pending and the GOTO is not finished
        # then increment the correction iterations counter
        self.guide_correction_iters += 1

        if self.guide_correction_iters > self.guide_correction_max_iters:
          # If we've exceeded the max iterations then just resume tracking
          if self.verbose:
            sys.stderr.write('Max correction iters exceeded. Resume tracking...\r\n')
          # DEC Servo Off
          self.dec_mod.ServoStopMotorOff()
          self.dec_axis_running = False
          self.dec_axis_goto = False
          self.dec_mod.ReadStatus(self.status_bits_dec)
          self.pos_dec = self.dec_mod.status_dict['pos']
          self.motor_current_dec = self.dec_mod.status_dict['cur_sense']
          self.pos_error_dec = self.dec_mod.status_dict['pos_error']
          # RA Servo Off
          self.ra_mod.ServoStopMotorOff()
          self.ra_axis_running = False
          self.ra_axis_tracking = False
          self.pec_tracking_status = 'ready'
          self.ra_axis_homing = False
          self.ra_axis_darv_pause = False
          self.ra_axis_darv_east = False
          self.ra_axis_darv_west = False
          self.ra_axis_goto = False
          self.ra_mod.ReadStatus(self.status_bits_ra)
          self.pos_ra = self.ra_mod.status_dict['pos']
          self.motor_current_ra = self.ra_mod.status_dict['cur_sense']
          self.pos_error_ra = self.ra_mod.status_dict['pos_error']
          self.guide_correction_iters = 0
          self.motion_q.put(('ra_track_start', None))


#      if self.guide_correction_pending and not (self.ra_axis_goto or self.dec_axis_goto):
      if self.guide_correction_pending and self.ra_axis_tracking and not self.dec_axis_goto:
        # If a correction is pending and the GOTO is finished
        # then we've just finished making the correction
        self.guide_correction_pending = False
        self.dRA = 0
        self.adj_RA = 0.0
        self.dDEC = 0
        # If we're guiding then trigger the next cycle:
        if self.autoguider_guiding:
          if self.verbose:
            sys.stderr.write('triggering autoguider...\r\n')
          self.autoguider_guide_star_drift()


      self.motion_running = self.ra_axis_running | self.ra_axis_tracking | self.dec_axis_running
      if not cmd == 'motion_continue':
        self.motion_q.task_done()


#############################


######################################
# Symmetric Threaded Client/Server
######################################

  # Start server thread 
  def server_start(self):
    self.input_start()
    self.motion_control_start()
    self.server_running = True

    self.threaded_server = ThreadedTCPServer((self.scopeserver_host, self.scopeserver_port), ThreadedTCPRequestHandler)

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    self.server_thread = threading.Thread(target=self.threaded_server.serve_forever)
    # Exit the server thread when the main thread terminates
#    self.server_thread.daemon = True
    self.server_thread.setDaemon(True)
    self.server_thread.start()

    sys.stderr.write('\r\nConnecting to Autoguider at %s:%d ...\r\n' %(self.autoguider_host,self.autoguider_port))
    self.autoguider_connect()

    sys.stderr.write('\r\nWaiting for a command, type q to quit...\r\n')
    sys.stderr.write('  h      -> print this help message\r\n')
    sys.stderr.write('  arrows -> slew N S E W\r\n')
    sys.stderr.write('  g      -> begin tracking\r\n')
    sys.stderr.write('  s      -> stop tracking\r\n')
    sys.stderr.write('  d      -> drift alignment routine DARV\r\n')
    sys.stderr.write('  p      -> report position\r\n')
    sys.stderr.write('  w      -> wifi connect with Autoguider\r\n')
    sys.stderr.write('  a      -> autoguider toggle auto-engage\r\n')
    sys.stderr.write('  t      -> autoguider latency test\r\n')
    sys.stderr.write('  r      -> reboot Autoguider rpi system\r\n')
    sys.stderr.write('  x      -> shutdown Autoguider rpi system\r\n')
    sys.stderr.write('  q      -> quit ScopeServer app\r\n\r\n')

    self.server_thread.join()


  # Stop server thread
  def server_stop(self):
    if self.server_running:
      self.server_running=False
      sys.stderr.write('ScopeServer shutting down...\r\n')

      # Shutdown NMC Network
      self.ra_mod.ServoStopMotorOff()
      self.dec_mod.ServoStopMotorOff()
      nmc_net.Shutdown()

      self.threaded_server.shutdown()
      self.server_thread.join()


  def send_status_to_autoguider(self):
    if not self.autoguider_connected:
      sys.stderr.write('Could Not Send ScopeServer Status\r\n')
      sys.stderr.write('Autoguider Not Connected\r\n')
      return

    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{scopeserver_status}'

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
        if response == 'ack':
          status = str(self.get_small_status())
          sock.sendall(bytes(status, 'utf-8'))
          response = str(sock.recv(1024), 'utf-8')
      except:
        response = ''
      finally:
        sock.close()

    if response != 'ack':
      sys.stderr.write('Send Status to Autoguider Failed\r\n')


  def reboot_autoguider(self):

    if not self.autoguider_connected:
      sys.stderr.write('Could Not Reboot Autoguider\r\n')
      sys.stderr.write('Autoguider Not Connected\r\n')
      return

    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{reboot_autoguider}'

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

    if response == 'ack':
      sys.stderr.write('Autoguider Rebooting...\r\n')
    else:
      sys.stderr.write('Autoguider Reboot Failed\r\n')


  def shutdown_autoguider(self):

    if not self.autoguider_connected:
      sys.stderr.write('Could Not Shutdown Autoguider\r\n')
      sys.stderr.write('Autoguider Not Connected\r\n')
      return

    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{shutdown_autoguider}'

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

    if response == 'ack':
      sys.stderr.write('Autoguider Shutdown...\r\n')
    else:
      sys.stderr.write('Autoguider Shutdown Failed\r\n')


  def autoguider_connect(self):
    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{scopeserver_connect}'

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

    if response == 'ack':
      self.autoguider_available = True
      self.autoguider_connected = True

#      if self.autoguider_guiding and not (self.ra_axis_goto or self.dec_axis_goto):
      if self.autoguider_guiding and self.ra_axis_tracking and not self.dec_axis_goto:
        # If autoguiding is enabled and GOTO is finished
        # we're recovering from a disconnect so trigger the next cycle:
        sys.stderr.write('Resuming Autoguiding...\r\n')
        self.guide_correction_pending = False
        self.dRA = 0
        self.adj_RA = 0.0
        self.dDEC = 0
        self.autoguider_guide_star_drift()

      if self.verbose:
        sys.stderr.write('Autoguider Connected\r\n')
    else:
      self.autoguider_connected = False
      if self.verbose:
        sys.stderr.write('Autoguider Not Connected\r\n')


  def get_autoguider_status(self):
    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{get_status}'

    # Create a socket (SOCK_STREAM means a TCP socket)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
      # Connect to server and send cmd
      try:
        sock.settimeout(0.1)
        sock.connect((HOST, PORT))
        sock.settimeout(None)
        sock.sendall(bytes(cmd, 'utf-8'))

        # Receive response from the server and finish
        response = str(sock.recv(1024), 'utf-8').split()
      except:
        response = []
      finally:
        sock.close()

    if len(response):
      self.autoguider_connected = True
      self.autoguider_status = response[0]
      self.autoguider_img_count = int(response[1])
      self.autoguider_gamma = float(response[2])
      self.autoguider_bp = int(response[3])
      self.autoguider_mag = int(response[4])
      self.autoguider_exposure = float(response[5])
      self.autoguider_interval = float(response[6])
#      sys.stderr.write('Autoguider Connected\r\n')
    else:
      self.autoguider_connected = False
      self.autoguider_status = 'idle'
      self.autoguider_img_count = 0
      self.autoguider_gamma = 1.0
      self.autoguider_bp = 0
      self.autoguider_mag = 1
      self.autoguider_exposure = 1.0
      self.autoguider_interval = 1.0 
#      sys.stderr.write('Autoguider Not Connected\r\n')


  def autoguider_guide_start(self):
    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{guide_start}'

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

    if response == 'ack':
      if self.verbose:
        sys.stderr.write('Autoguider Guiding Started\r\n')
    else:
      if self.verbose:
        sys.stderr.write('Autoguider Error: Could Not Start Guiding\r\n')


  def autoguider_guide_stop(self):
    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{guide_stop}'

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

    if response == 'ack':
      if self.verbose:
        sys.stderr.write('Autoguider Guiding Stopped\r\n')
    else:
      if self.verbose:
        sys.stderr.write('Autoguider Error: Could Not Stop Guiding\r\n')


  def autoguider_guide_star_drift(self):
    HOST = self.autoguider_host
    PORT = self.autoguider_port
    cmd = '{guide_star_drift}'

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

    if response == 'ack':
      if self.verbose:
        sys.stderr.write('ScopeServer Received Ack: Autoguider Performing Guide Star Drift\r\n')
    else:
      sys.stderr.write('ScopeServer No Ack: Autoguider Could Not Perform Guide Star Drift: %s \r\n' % (response))
      sys.stderr.write('Attempting to Reconnect...\r\n')
      self.motion_q.put(('ag_connect', None))


#############################
# Keyboard Input Processor
#############################

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
          self.motion_q.put(('dec_slew_stop', None))
        else:
          self.motion_q.put(('dec_slew_start', -1*self.meridian_mode))
#        sys.stderr.write('  moved N to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # S, down arrow
      elif k=='\x1b[B':
        if self.dec_axis_running:
          self.motion_q.put(('dec_slew_stop', None))
        else:
          self.motion_q.put(('dec_slew_start', 1*self.meridian_mode))
#        sys.stderr.write('  moved S to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # E, right arrow
      elif k=='\x1b[C':
        if self.ra_axis_running:
          self.motion_q.put(('ra_slew_stop', None))
        elif self.ra_axis_tracking:
          self.motion_q.put(('ra_slew_start', 1))
        else:
          self.motion_q.put(('ra_slew_start', 1))
#        sys.stderr.write('  moved E to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # W, left arrow
      elif k=='\x1b[D':
        if self.ra_axis_running:
          self.motion_q.put(('ra_slew_stop', None))
        elif self.ra_axis_tracking:
          self.motion_q.put(('ra_slew_start', -1))
        else:
          self.motion_q.put(('ra_slew_start', -1))
#        sys.stderr.write('  moved W to:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Start RA axis tracking
      elif k=='g':
        if not self.ra_axis_tracking:
          sys.stderr.write('  Start Tracking RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
          ra_time_str = self.get_ra_time()
          self.target_ra_time = ra_time_str
          self.set_target_ra_time_array(ra_time_str)
          dec_angle = self.get_dec_angle()
          self.target_dec_angle = dec_angle
          self.target_dec_pos = self.dec_angle_to_pos(dec_angle)
          self.motion_q.put(('ra_track_start', None))

      # Stop RA axis tracking
      elif k=='s':
        if self.ra_axis_tracking:
          self.motion_q.put(('ra_track_stop', None))
          sys.stderr.write('  Stopped Tracking RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # Start Drift Alignment Robert Vice (DARV) routine on RA axis, go East
      elif k=='d':
        if not self.ra_axis_tracking:
          sys.stderr.write('  Starting Drift Alignment Slew on RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
          self.motion_q.put(('ra_darv_east', None))

      # Report Position
      elif k=='p':
        self.motion_q.put(('update_pos', True))
        sys.stderr.write('  Current Pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))

      # WiFi connect with Autoguider
      elif k=='w':
        self.motion_q.put(('ag_connect', None))

      # Autoguider toggle autoengage
      elif k=='a':
        self.autoguider_autoengage = not self.autoguider_autoengage

      # Autoguider Latency Test
      elif k=='t':
        self.motion_q.put(('autoguider_latency_test', None))

      # Reboot Autoguider RPI System
      elif k=='r':
        self.motion_q.put(('reboot_autoguider', None))

      # Shutdown Autoguider RPI System
      elif k=='x':
        self.motion_q.put(('shutdown_autoguider', None))

      # Shutdown the server
      elif k=='q':
        self.input_running = False

      # Print help message
      elif k=='h':
        sys.stderr.write('\r\n>>> ' + str(k) + '\r\n')
        sys.stderr.write('  h      -> print this help message\r\n')
        sys.stderr.write('  arrows -> slew N S E W\r\n')
        sys.stderr.write('  g      -> begin tracking\r\n')
        sys.stderr.write('  s      -> stop tracking\r\n')
        sys.stderr.write('  d      -> drift alignment routine DARV\r\n')
        sys.stderr.write('  p      -> report position\r\n')
        sys.stderr.write('  w      -> wifi connect with Autoguider\r\n')
        sys.stderr.write('  a      -> autoguider toggle auto-engage\r\n')
        sys.stderr.write('  t      -> autoguider latency test\r\n')
        sys.stderr.write('  r      -> reboot Autoguider rpi system\r\n')
        sys.stderr.write('  x      -> shutdown Autoguider rpi system\r\n')
        sys.stderr.write('  q      -> quit ScopeServer app\r\n\r\n')
      else:
        sys.stderr.write('>>> ' + str(k) + ' \r\n')
        sys.stderr.write('Unknown key! Please type q to quit.\r\n')
        sys.stderr.write('  h      -> print this help message\r\n')
        sys.stderr.write('  arrows -> slew N S E W\r\n')
        sys.stderr.write('  g      -> begin tracking\r\n')
        sys.stderr.write('  s      -> stop tracking\r\n')
        sys.stderr.write('  d      -> drift alignment routine DARV\r\n')
        sys.stderr.write('  p      -> report position\r\n')
        sys.stderr.write('  w      -> wifi connect with Autoguider\r\n')
        sys.stderr.write('  a      -> autoguider toggle auto-engage\r\n')
        sys.stderr.write('  t      -> autoguider latency test\r\n')
        sys.stderr.write('  r      -> reboot Autoguider rpi system\r\n')
        sys.stderr.write('  x      -> shutdown Autoguider rpi system\r\n')
        sys.stderr.write('  q      -> quit ScopeServer app\r\n\r\n')
    self.server_stop()


###############################################################
# ScopeServer BlueShift Telescope Protocol Command Processor
###############################################################

  def process_scopeserver_cmd(self,cmd):

    response = None

    cmd = cmd[1:-1]

    if cmd.split()[0] == 'jog_pos':
      jog_ra = int(cmd.split()[1])
      jog_dec = int(cmd.split()[2])
      response = '@%s: jogging: %d %d' % (str(time.time()), jog_ra, jog_dec)
#      self.motion_q.put(('jog_pos', (jog_ra, jog_dec)))

    elif cmd == 'autoguider_connect':
      self.autoguider_available = True
      self.autoguider_connected = True
      response = 'ack'
      if self.verbose:
        sys.stderr.write('Autoguider Connected\r\n')
#      if self.autoguider_guiding and not (self.ra_axis_goto or self.dec_axis_goto):
      if self.autoguider_guiding and self.ra_axis_tracking and not self.dec_axis_goto:
        # If autoguiding is enabled and GOTO is finished
        # we're recovering from a disconnect so trigger the next cycle:
        sys.stderr.write('Resuming Autoguiding\r\n')
        self.guide_correction_pending = False
        self.dRA = 0
        self.adj_RA = 0.0
        self.dDEC = 0
        self.autoguider_guide_star_drift()

    # Autoguider toggle autoengage
    elif cmd == 'toggle_autoguider_auto':
      self.autoguider_autoengage = not self.autoguider_autoengage
      response = 'ack'

    elif cmd == 'toggle_guiding':
      if self.autoguider_connected:
        if self.autoguider_guiding:
          self.motion_q.put(('autoguider_guide_stop', None))
        elif self.ra_axis_tracking:
          self.motion_q.put(('autoguider_guide_start', None))
      response = 'ack'

    elif cmd.split()[0] == 'guide_star_correction':
#      if self.autoguider_guiding:
      if self.ra_axis_tracking:
        # Perform guide_star_correction by adjusting the goto coordinates
        self.dRA = -int(cmd.split()[1])
        self.dDEC = int(cmd.split()[2])*self.meridian_mode
        reset_target = eval(cmd.split()[3])
        dec_angle_rad = self.dec_angle_to_radians(self.get_dec_angle())
        #FIXME: why int() here?
        self.dRA = int(self.dRA/math.cos(dec_angle_rad))
        if reset_target:
          # Set cur pos_ra as current target
          ra_time_str = self.get_ra_time()
          self.target_ra_time = ra_time_str
          self.set_target_ra_time_array(ra_time_str)
          # Set cur pos_dec as current target
          dec_angle = self.get_dec_angle()
          self.target_dec_angle = dec_angle
          self.target_dec_pos = self.dec_angle_to_pos(dec_angle)
        self.guide_correction_pending = True
        self.adj_RA = self.dRA/self.sidereal_rate
        self.adj_target_ra_time_array(self.adj_RA)
        self.target_dec_pos += self.dDEC
        if self.verbose:
          sys.stderr.write('Guide Correction dRA: %d  adj_RA: %g  dDEC: %d\r\n' % (self.dRA, self.adj_RA, self.dDEC))
        if reset_target:
          # This is a 'move to' command
          self.guide_correction_iters = 0
          self.guide_correction_max_iters = 1e6
#          self.ra_axis_goto_precision = 'low'
          self.ra_axis_goto_precision = 'high'
          self.motion_q.put(('goto_target_start','center_rate'))
        else:
          # This is a true guide star correction
          self.guide_correction_iters = 0
          self.guide_correction_max_iters = 100
          self.ra_axis_goto_precision = 'high'
          self.motion_q.put(('goto_target_start','correction_rate'))
      response = 'ack'

    elif cmd == 'jog_target_north':
      # Jog tracking by adjusting the goto coordinates
      self.dRA = 0
      self.dDEC = 3334*self.meridian_mode  # ~240 arcsec
      self.adj_RA = self.dRA/self.sidereal_rate
      self.guide_correction_pending = True
      self.adj_target_ra_time_array(self.adj_RA)
      self.target_dec_pos += self.dDEC
      self.motion_q.put(('goto_target_start',None))
      response = 'ack'

    elif cmd == 'jog_target_south':
      # Jog tracking by adjusting the goto coordinates
      self.dRA = 0
      self.dDEC = -3334*self.meridian_mode  # ~240 arcsec
      self.adj_RA = self.dRA/self.sidereal_rate
      self.guide_correction_pending = True
      self.adj_target_ra_time_array(self.adj_RA)
      self.target_dec_pos += self.dDEC
      self.motion_q.put(('goto_target_start',None))
      response = 'ack'

    elif cmd == 'jog_target_east':
      # Jog tracking by adjusting the goto coordinates
      self.dRA = 4267  # ~240 arcsec
      self.dDEC = 0
      self.adj_RA = self.dRA/self.sidereal_rate
      self.guide_correction_pending = True
      self.adj_target_ra_time_array(self.adj_RA)
      self.target_dec_pos += self.dDEC
      self.motion_q.put(('goto_target_start',None))
      response = 'ack'

    elif cmd == 'jog_target_west':
      # Jog tracking by adjusting the goto coordinates
      self.dRA = -4267  # ~240 arcsec
      self.dDEC = 0.0  
      self.adj_RA = self.dRA/self.sidereal_rate
      self.guide_correction_pending = True
      self.adj_target_ra_time_array(self.adj_RA)
      self.target_dec_pos += self.dDEC
      self.motion_q.put(('goto_target_start',None))
      response = 'ack'

    elif cmd == 'get_scopeserver_time':
      self.autoguider_available = True
      self.autoguider_connected = True
      response = 'scopeserver_time %s' % (str(time.time_ns()))
#      sys.stderr.write('Current Time:  %s\r\n' % (time.strftime('%a %b %d %H:%M:%S %Z %Y')))

    elif cmd == 'reboot_autoguider':
      self.motion_q.put(('reboot_autoguider', None))
      response = 'ack'

    elif cmd == 'shutdown_autoguider':
      self.motion_q.put(('shutdown_autoguider', None))
      response = 'ack'

    elif cmd == 'get_status':
      response = str(self.get_status())

    elif cmd == 'get_small_status':
      response = str(self.get_small_status())

    elif cmd == 'reset_server':
      response = 'ack'

    elif cmd == 'shutdown_server':
      response = 'ack'
      self.server_stop()

    elif cmd == 'reboot_system':
      response = 'ack'

    elif cmd == 'shutdown_system':
      response = 'ack'

    elif cmd == 'meridian_flip':
      if self.ra_axis_running:
        self.motion_q.put(('ra_slew_stop', None))
      if self.dec_axis_running:
        self.motion_q.put(('dec_slew_stop', None))
      self.meridian_flip()
      if self.autoguider_connected:
        self.send_status_to_autoguider()
      response = 'ack'

    elif cmd == 'set_ra_index_pos':
      if self.ra_axis_running:
        self.motion_q.put(('ra_slew_stop', None))
      if self.dec_axis_running:
        self.motion_q.put(('dec_slew_stop', None))
      self.motion_q.put(('set_ra_index_pos_step_1', None))
      response = 'ack'

    elif cmd == 'toggle_pec':
      self.motion_q.put(('toggle_pec', None))
      response = 'ack'

    # Toggle RA axis tracking
    elif cmd == 'toggle_slew_track':
      if not self.ra_axis_tracking:
        # Start RA axis tracking
        if self.verbose:
          sys.stderr.write('  Start Tracking RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
        ra_time_str = self.get_ra_time()
        self.target_ra_time = ra_time_str
        self.set_target_ra_time_array(ra_time_str)
        dec_angle = self.get_dec_angle()
        self.target_dec_angle = dec_angle
        self.target_dec_pos = self.dec_angle_to_pos(dec_angle)
#        if self.dec_axis_running:
#          self.motion_q.put(('dec_slew_stop', None))
        self.motion_q.put(('ra_track_start', None))
      else:
        # Stop RA axis tracking
        self.motion_q.put(('ra_track_stop', None))
        if self.dec_axis_running:
          self.motion_q.put(('dec_slew_stop', None))
        if self.verbose:
          sys.stderr.write('  Stopped Tracking RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
      response = 'ack'

    elif cmd == 'toggle_darv':
      if self.ra_axis_running:
        if self.verbose:
          sys.stderr.write('  Canceling Drift Alignment Slew on RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
        self.ra_axis_darv_pause = False
        self.ra_axis_darv_east = False
        self.ra_axis_darv_west = False
        self.motion_q.put(('ra_slew_stop', None))
      if self.ra_axis_tracking:
        if self.verbose:
          sys.stderr.write('  Starting Drift Alignment Slew on RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
        self.ra_axis_darv_pause = False
        self.ra_axis_darv_east = False
        self.ra_axis_darv_west = False
        self.motion_q.put(('ra_slew_stop', None))
        self.motion_q.put(('ra_darv_pause', None))
#        self.motion_q.put(('ra_darv_east', None))
      else:
        if self.verbose:
          sys.stderr.write('  Starting Drift Alignment Slew on RA Axis at RA pos:  %.9g %.9g  %s %s\r\n' % (self.pos_ra, self.pos_dec, self.get_ra_time(), self.get_dec_angle()))
        self.motion_q.put(('ra_darv_pause', None))
#        self.motion_q.put(('ra_darv_east', None))
      response = 'ack'

    elif cmd == 'toggle_slew_north':
      if not self.dec_axis_running:
        self.process_lx200_cmd(':Mn#')
      else:
        self.process_lx200_cmd(':Qn#')
      response = 'ack'

    elif cmd == 'toggle_slew_south':
      if not self.dec_axis_running:
        self.process_lx200_cmd(':Ms#')
      else:
        self.process_lx200_cmd(':Qs#')
      response = 'ack'

    elif cmd == 'toggle_slew_east':
      if not self.ra_axis_running:
        self.process_lx200_cmd(':Me#')
      else:
        self.process_lx200_cmd(':Qe#')
      response = 'ack'

    elif cmd == 'toggle_slew_west':
      if not self.ra_axis_running:
        self.process_lx200_cmd(':Mw#')
      else:
        self.process_lx200_cmd(':Qw#')
      response = 'ack'

    else:
      pass

    return response


#######################################################
# LX200 Telescope Control Protocol Command Processor
#######################################################

  def process_lx200_cmd(self,cmd):

    response = None


    if (cmd != ':GD#') & (cmd != ':GR#'):
      if self.verbose:
        sys.stderr.write('  Processing LX200 cmd: %s\r\n' % (cmd))

    if cmd == ':GD#':  # Get Dec angle:  sDD*MM'SS#
      self.motion_q.put(('update_pos', None))
      response = '%s#' % self.get_dec_angle()
#      sys.stderr.write('  Server responding: %s\r\n' % (response))

    elif cmd == ':GR#': # Get RA time:  HH:MM:SS#
      self.motion_q.put(('update_pos', None))
      response = '%s#' % self.get_ra_time()
#      sys.stderr.write('  Server responding: %s\r\n' % (response))

    elif cmd[0:3] == ':St':  # Set site latitude:  sDD*MM#
      dd, mm = cmd[3:-1].split('*')
      dd_sign = float(dd[0] + '1')
      if not self.gpsd_connected:
        self.site_latitude = float(dd) + dd_sign*float(mm)/60.0
      response = '1'

    elif cmd[0:3] == ':Sg':  # Set site longitude: DDD*MM#
      ddd, mm = cmd[3:-1].split('*')
      if not self.gpsd_connected:
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
      resume_tracking = self.ra_axis_tracking
      self.ra_axis_goto = False
      self.motion_q.put(('ra_slew_stop', None))
      self.motion_q.put(('dec_slew_stop', None))
      self.motion_q.put(('align_to_target', resume_tracking))
      response = "#"

    elif cmd == ':RS#':
      self.slew_rate = self.slew_rate_max
      self.servo_ra_slew_rate = int(self.slew_rate*0.000512*2**16)
      self.servo_dec_slew_rate = int(self.slew_rate*0.000512*2**16)

    elif cmd == ':RM#':
      self.slew_rate = self.slew_rate_find
      self.servo_ra_slew_rate = int(self.slew_rate*0.000512*2**16)
      self.servo_dec_slew_rate = int(self.slew_rate*0.000512*2**16)

    elif cmd == ':RC#':
      self.slew_rate = self.slew_rate_center
      self.servo_ra_slew_rate = int(self.slew_rate*0.000512*2**16)
      self.servo_dec_slew_rate = int(self.slew_rate*0.000512*2**16)

    elif cmd == ':RG#':
      self.slew_rate = self.slew_rate_track
      self.servo_ra_slew_rate = int(self.slew_rate*0.000512*2**16)
      self.servo_dec_slew_rate = int(self.slew_rate*0.000512*2**16)

    elif cmd[0:3] == ':Sr':  # Set target object RA:  HH:MM:SS#
      self.target_ra_time = cmd[3:-1]
      self.set_target_ra_time_array(cmd[3:-1])
      response = '1'

    elif cmd[0:3] == ':Sd':  # Set target object Dec: sDD*MM:SS#
      self.target_dec_angle = cmd[3:-1]
      self.target_dec_pos = self.dec_angle_to_pos(cmd[3:-1])
      response = '1'

    elif cmd == ':MS#':  # Slew to target
      if self.ra_axis_tracking:
        self.motion_q.put(('ra_slew_stop', None))
      else:
        self.ra_axis_goto_precision = 'high'
        self.motion_q.put(('goto_target_start',None))
      response = '0'

    elif cmd == ':Mn#':  # Start move North
      self.motion_q.put(('dec_slew_start',-1*self.meridian_mode))

    elif cmd == ':Ms#':  # Start move South
      self.motion_q.put(('dec_slew_start',1*self.meridian_mode))

    elif cmd == ':Me#':  # Start move East
      self.motion_q.put(('ra_slew_start',1))

    elif cmd == ':Mw#':  # Start move West
      self.motion_q.put(('ra_slew_start',-1))

    elif cmd == ':Qn#':  # Stop move North
      self.motion_q.put(('dec_slew_stop', None))

    elif cmd == ':Qs#':  # Stop move South
      self.motion_q.put(('dec_slew_stop', None))

    elif cmd == ':Qe#':  # Stop move East
      self.motion_q.put(('ra_slew_stop', None))
      if self.ra_axis_tracking:
        self.motion_q.put(('ra_track_start', None))

    elif cmd == ':Qw#':  # Stop move West
      self.motion_q.put(('ra_slew_stop', None))
      if self.ra_axis_tracking:
        self.motion_q.put(('ra_track_start', None))

    elif cmd == ':Q#':  # Stop all slews
      self.ra_axis_goto = False
      self.dec_axis_goto = False
      self.motion_q.put(('ra_slew_stop', None))
      self.motion_q.put(('dec_slew_stop', None))

    else:
      pass
  
    return response


#############################
# ScopeServer Main Routine
#############################

if (__name__ == '__main__'):

#  if (len(sys.argv)<3):
#    print('\nUsage: %s server_address port\n' % (sys.argv[0]))
#    print('   Example: %s 10.0.1.15 4030\n' % (sys.argv[0]))
#    sys.exit()
#  server_address = sys.argv[1]
#  port = int(sys.argv[2])

  pid = os.getpid()
  try:
    pidfile = open('/var/run/scopeserver_control.pid','w')
    pidfile.write('%d\n' % (pid))
    pidfile.close()
  except:
    pass

  server_address = get_ip()
  port = 54030

  print("\nStarting ScopeServer at %s:%d" % (server_address, port))

  scope = scope_server(server_address, port)
  scope.server_start()

