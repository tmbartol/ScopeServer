from django.shortcuts import render
from django.http import JsonResponse
from pdb import set_trace as debug

# Create your views here.

import datetime
import time
import socket
import sys
import traceback

# Send a command to scope_server socket and receive response
def send_scopeserver_cmd(cmd):
  HOST, PORT = "0.0.0.0", 4030

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
    if slew:
      mouse = request.POST.get("mouse") == "mousedown"

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

      result['status'] = "Success"
      return(JsonResponse(result))
    

    action = request.POST.get("action", "")
    try:
      if action == "serverStatus":
        scope_status = send_scopeserver_cmd('{get_status}')
        status_dict = eval(scope_status)
        result['#gps_location_lat'] = status_dict['site_latitude']
        result['#gps_location_lon'] = status_dict['site_longitude']
        result['#local_time'] = status_dict['site_local_time'][:-5]
        result['#utc_time'] = status_dict['site_utc_time'][:-5]
        result['#scope_position_dec'] = '%s (%*.*f)' % (status_dict['dec_angle'], 8, 2, status_dict['dec_pos'])
        result['#scope_position_ra'] = '%s (%*.*f)' % (status_dict['ra_time'], 8, 2, status_dict['ra_pos'])
        result['#target_position_dec'] = '%s (%*.*f)' % (status_dict['target_dec_angle'], 8, 2, status_dict['target_dec_pos'])
        result['#target_position_ra'] = '%s (%*.*f)' % (status_dict['target_ra_time'], 8, 2, status_dict['target_ra_pos'])
        result['#motor_current_ra'] = status_dict['motor_current_ra']
        result['#motor_current_dec'] = status_dict['motor_current_dec']
        result['status'] = "Success"
    except Exception as e:
      result['status'] = "Failure"
      result['msg'] = str(e)
    finally:
      return(JsonResponse(result))
  elif request.method == "POST":
    action = request.POST.get("action")
    if action == "reboot":
      print("REBOOTING BY BUTTON PRESS")
    elif action == "shutdown":
      print("SHUTTING DOWN BY BUTTON PRESS")

  return(render(request, "server/control.html", locals()))

