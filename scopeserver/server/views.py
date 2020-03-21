from django.shortcuts import render
from django.http import JsonResponse
from pdb import set_trace as debug

# Create your views here.

import datetime
import time
import socket
import os
import sys
import traceback
import subprocess


# Get IP address
import netifaces as ni
def get_ip(iface = 'wlan0'):
  try:
    ni.ifaddresses(iface)
    ip = ni.ifaddresses(iface)[ni.AF_INET][0]['addr']
  except:
    ip = '127.0.0.1'
  return ip


# Send a command to scope_server socket and receive response
def send_scopeserver_cmd(cmd):
#  HOST, PORT = "0.0.0.0", 4030
#  HOST, PORT = "10.0.1.14", 4030
#  HOST, PORT = "192.168.50.5", 4030
#  HOST, PORT = "10.0.1.15", 4030
#  HOST, PORT = "169.254.135.86", 4030
 
  HOST = get_ip()
  PORT = 54030

  # Create a socket (SOCK_STREAM means a TCP socket)
  scope_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  try:
      # Connect to server and send data
      scope_socket.connect((HOST, PORT))
      scope_socket.sendall(cmd.encode('ascii'))

      # Receive data from the server and shut down
      response = scope_socket.recv(1024).decode('ascii')
  except:
      tracebackStr = traceback.format_exc()
      raise(Exception("ScopeServer not found at {0}:{1}\n\n{2}".format(HOST, PORT, tracebackStr)))  #This is new. 
  finally:
      scope_socket.close()
#  print("Sent:     {}".format(cmd))
#  print("Received: {}".format(response))
  return response #Response will not exist if there is an exception. 


def control(request):

  if request.is_ajax():
    result = {}
    slew = request.POST.get("slew")
    print(slew)
    if slew:
      mouse = (request.POST.get("mouse") == "mousedown") or (request.POST.get("mouse") == "touchstart")
      if slew == "slew_north":
        if mouse:
          print("Start Slew North")
          response = send_scopeserver_cmd(':Mn#')
        else:
          print("Stop Slew North")
          response = send_scopeserver_cmd(':Qn#')

      elif slew == "slew_south":
        if mouse:
          print("Start Slew South")
          response = send_scopeserver_cmd(':Ms#')
        else:
          print("Stop Slew South")
          response = send_scopeserver_cmd(':Qs#')

      elif slew == "slew_east":
        if mouse:
          print("Start Slew East")
          response = send_scopeserver_cmd(':Me#')
        else:
          print("Stop Slew East")
          response = send_scopeserver_cmd(':Qe#')

      elif slew == "slew_west":
        if mouse:
          print("Start Slew West")
          response = send_scopeserver_cmd(':Mw#')
        else:
          print("Stop Slew West")
          response = send_scopeserver_cmd(':Qw#')

      elif slew == "slew_guide":
          print("Toggle Guide Mode")
          response = send_scopeserver_cmd('{toggle_slew_guide}')

      result['status'] = "Success"
      return(JsonResponse(result))

    ''' This is the toggle method
    if slew:
      mouse = request.POST.get("mouse") == "mousedown"

      if slew == "slew_north":
        print("Toggle Slew North")
        response = send_scopeserver_cmd('{toggle_slew_north}')

      elif slew == "slew_south":
        print("Toggle Slew South")
        response = send_scopeserver_cmd('{toggle_slew_south}')

      elif slew == "slew_east":
        print("Toggle Slew East")
        response = send_scopeserver_cmd('{toggle_slew_east}')

      elif slew == "slew_west":
        print("Toggle Slew West")
        response = send_scopeserver_cmd('{toggle_slew_west}')

      elif slew == "slew_guide":
          print("Toggle Guide Mode")
          response = send_scopeserver_cmd('{toggle_slew_guide}')

      result['status'] = "Success"
      return(JsonResponse(result))
    '''

    action = request.POST.get("action", "")
    try:
      if action == "serverStatus":
        scope_status = send_scopeserver_cmd('{get_status}')
        status_dict = eval(scope_status)
        result['#gps_location_lat'] = status_dict['site_latitude']
        result['#gps_location_lon'] = status_dict['site_longitude']
        lt = status_dict['site_local_time']
        lt1 = lt[:-6]
        lt2 = str(round(float(lt[-7:])*2)/2)[-1]
        result['#local_time'] = lt1+lt2
        ut = status_dict['site_utc_time']
        ut1 = ut[:-6]
        ut2 = str(round(float(ut[-7:])*2)/2)[-1]
        result['#utc_time'] = ut1+ut2
        result['#t_acc'] = 't_acc:  %.4gus  :  %.4gus' % (1000*status_dict['t_offset'], 1000*status_dict['t_jitter'])
        result['#meridian_mode'] = 'mflp:  %d' % (status_dict['meridian_mode'])
        result['#scope_position_dec'] = '%s (%d)' % (status_dict['dec_angle'], status_dict['dec_pos'])
        result['#scope_position_ra'] = '%s (%d)' % (status_dict['ra_time'], status_dict['ra_pos'])
        result['#target_position_dec'] = '%s (%d)' % (status_dict['target_dec_angle'], status_dict['target_dec_pos'])
        result['#target_position_ra'] = '%s (%d)' % (status_dict['target_ra_time'], status_dict['target_ra_pos'])
        result['#motor_current_dec'] = '%d' % (status_dict['motor_current_dec'])
        result['#motor_current_ra'] = '%d' % (status_dict['motor_current_ra'])
        result['#pos_error_dec'] = '%d' % (status_dict['pos_error_dec'])
        result['#pos_error_ra'] = '%d' % (status_dict['pos_error_ra'])
        result['status'] = "Success"
      elif action == "darv":
        print("Toggle DARV")
        response = send_scopeserver_cmd('{toggle_darv}')
        result['status'] = "Success"
      elif action == "meridianflip":
        print("Meridian Flip")
        response = send_scopeserver_cmd('{meridian_flip}')
        result['status'] = "Success"
      elif action == "gpsreset":
        print("RESETTING GPS BY BUTTON PRESS")
        time.sleep(1)
        os.system('nohup sudo /home/pi/src/scopeserver-git/scopeserver/scopeserver_control_gpsreset.sh &')
      elif action == "ssreset":
        print("RESETTING SCOPE SERVER BY BUTTON PRESS")
        time.sleep(1)
        os.system('nohup sudo /home/pi/src/scopeserver-git/scopeserver/scopeserver_control_ssreset.sh &')
      elif action == "reboot":
        print("REBOOTING BY BUTTON PRESS")
        time.sleep(1)
        subprocess.check_output(["sudo reboot"],shell=True)
      elif action == "shutdown":
        print("SHUTTING DOWN BY BUTTON PRESS")
        time.sleep(1)
        subprocess.check_output(["sudo shutdown now"],shell=True)

    except Exception as e:
      result['status'] = "Failure"
      result['msg'] = str(e)
    finally:
      return(JsonResponse(result))
  elif request.method == "POST":
    pass
    '''
    action = request.POST.get("action")
    if action == "reboot":
      print("REBOOTING BY BUTTON PRESS")
      time.sleep(1)
      subprocess.check_output(["sudo reboot"],shell=True)
    elif action == "shutdown":
      print("SHUTTING DOWN BY BUTTON PRESS")
      time.sleep(1)
      subprocess.check_output(["sudo shutdown now"],shell=True)
    '''

  return(render(request, "server/control.html", locals()))

