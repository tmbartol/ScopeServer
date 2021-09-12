from django.shortcuts import render
from django.http import JsonResponse

# Create your views here.

import socket


# Get IP address
import netifaces as ni
def get_ip(iface = 'wlan0'):
  try:
    ni.ifaddresses(iface)
    ip = ni.ifaddresses(iface)[ni.AF_INET][0]['addr']
  except:
    ip = '127.0.0.1'
  return ip


scopeserver_host = get_ip()
scopeserver_port = 54030
host_dict = {}
host_dict['10.0.1.20'] = '10.0.1.23'
host_dict['10.0.1.23'] = '10.0.1.24'
host_dict['192.168.50.5'] = '192.168.50.10'
autoguider_host = host_dict[scopeserver_host]
autoguider_port = 54040
autoguider_ud_port = 54050


# Send a command to scope_server socket and receive response
def send_scopeserver_cmd(cmd):

  # Create a socket (SOCK_STREAM means a TCP socket)
  scope_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  response = ''
  try:
      # Connect to server and send data
      scope_socket.settimeout(0.1)
      scope_socket.connect((scopeserver_host, scopeserver_port))
      scope_socket.settimeout(None)
      scope_socket.sendall(cmd.encode('utf-8'))

      # Receive data from the server and shut down
      response = scope_socket.recv(1024).decode('utf-8')
  except:
#      import traceback
#      tracebackStr = traceback.format_exc()
#      raise(Exception("ScopeServer not found at {0}:{1}\n\n{2}".format(HOST, PORT, tracebackStr)))  #This is new. 
      pass
  finally:
      scope_socket.close()
#  print("Sent:     {}".format(cmd))
#  print("Received: {}".format(response))
  return response #Response will not exist if there is an exception. 


# Send a command to autoguider socket and receive response
def send_autoguider_cmd(cmd):

  buf_size = int(2**16)
#  buf_size = 2**16
#  buf_size = 2**13
#  buf_size = 2**12
#  buf_size = 1024

  # Create a socket (SOCK_STREAM means a TCP socket)
  guider_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  response = ''
  try:
    # Connect to server and send data
    guider_socket.settimeout(0.1)
    guider_socket.connect((autoguider_host, autoguider_port))
    guider_socket.settimeout(None)
    guider_socket.sendall(cmd.encode('utf-8'))

    # Receive data from the server and shut down
    response = guider_socket.recv(1024).decode('utf-8')
    if cmd == '{get_view}':
      length = int(response) 
      response = b''
      while len(response) < length:
        # doing it in batches is generally better than trying
        # to do it all in one go
        to_read = length - len(response)
        response += guider_socket.recv(buf_size if to_read > buf_size else to_read)
#      print('get_view received: %d' % (length))
      response = response.decode('utf-8')
      ack = 'ack'
      guider_socket.sendall(ack.encode('utf-8'))
      guider_socket.close()
    else:
      if response != 'ack':
        response = ''
  except:
    response = ''
  finally:
    guider_socket.close()
  return response  #Response will be None if there is an exception. 


# Send a command to autoguider updown server socket and receive response
def send_autoguider_ud_cmd(cmd):

  # Create a socket (SOCK_STREAM means a TCP socket)
  updown_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  response = ''
  try:
    # Connect to server and send data
    updown_socket.settimeout(0.1)
    updown_socket.connect((autoguider_host, autoguider_ud_port))
    updown_socket.settimeout(None)
    updown_socket.sendall(cmd.encode('utf-8'))

    # Receive data from the server and shut down
    response = updown_socket.recv(1024).decode('utf-8')
  except:
    pass
  finally:
    updown_socket.close()
  return response  #Response will be None if there is an exception. 


def control(request):

  if request.is_ajax():
    result = {}
    slew = request.POST.get("slew")
#    print(slew)
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

      elif slew == "jog_target_north":
          print("Jog Target North")
          response = send_scopeserver_cmd('{jog_target_north}')

      elif slew == "jog_target_south":
          print("Jog Target South")
          response = send_scopeserver_cmd('{jog_target_south}')

      elif slew == "jog_target_east":
          print("Jog Target East")
          response = send_scopeserver_cmd('{jog_target_east}')

      elif slew == "jog_target_west":
          print("Jog Target West")
          response = send_scopeserver_cmd('{jog_target_west}')

      elif slew == "slew_track":
          print("Toggle Guide Mode")
          response = send_scopeserver_cmd('{toggle_slew_track}')

      result['status'] = "Success"
      return(JsonResponse(result))


    action = request.POST.get("action", "")
    try:
      if action == "serverStatus":
        scope_status = send_scopeserver_cmd('{get_status}')
        status_dict = eval(scope_status)
        if status_dict['gps_status']:
          result['#gps_location_lat'] = '%.5f -- GPS Fix' % (status_dict['site_latitude'])
        else:
          result['#gps_location_lat'] = '%.5f -- No GPS Fix' % (status_dict['site_latitude'])
        result['#gps_location_lon'] = '%.5f' % (status_dict['site_longitude'])
        lt = status_dict['site_local_time']
        lt1 = lt[:-6]
        lt2 = str(round(float(lt[-7:])*2)/2)[-1]
        result['#local_time'] = lt1+lt2
        ut = status_dict['site_utc_time']
        ut1 = ut[:-6]
        ut2 = str(round(float(ut[-7:])*2)/2)[-1]
        result['#utc_time'] = ut1+ut2
        result['#t_acc'] = 't_acc:  %.4gus  :  %.4gus' % (1000*status_dict['t_offset'], 1000*status_dict['t_jitter'])
        result['#meridian_mode'] = 'm: %s | PECv: %d %s' % (status_dict['meridian_mode'], status_dict['pec_rate'], status_dict['pec_tracking'])
        result['#scope_position_dec'] = '%s (%d)' % (status_dict['dec_angle'], status_dict['dec_pos'])
        result['#scope_position_ra'] = '%s (%d,%d)' % (status_dict['ra_time'], status_dict['ra_pos'], status_dict['pec_idx'])
        result['#target_position_dec'] = '%s (%d)' % (status_dict['target_dec_angle'], status_dict['target_dec_pos'])
        result['#target_position_ra'] = '%s (%d)' % (status_dict['target_ra_time'], status_dict['target_ra_pos'])
        result['#motor_current_dec'] = '%d' % (status_dict['motor_current_dec'])
        result['#motor_current_ra'] = '%d' % (status_dict['motor_current_ra'])
        result['#pos_error_dec'] = '%d' % (status_dict['pos_error_dec'])
        result['#pos_error_ra'] = '%d' % (status_dict['pos_error_ra'])
        result['#autoguider_connected'] = '%s' % (status_dict['autoguider_connected'])
        result['#autoguider_status'] = '%s' % (status_dict['autoguider_status'])
        result['#gamma_val'] = '%s' % (status_dict['gamma_val_str'])
        result['#gamma_val_str'] = '%s' % (status_dict['gamma_val_str'])
        result['#bp_val'] = '%s' % (status_dict['bp_val_str'])
        result['#bp_val_str'] = '%s' % (status_dict['bp_val_str'])
        result['#mag_val'] = '%s' % (status_dict['mag_val_str'])
        result['#mag_val_str'] = '%s' % (status_dict['mag_val_str'])
        result['#exposure_val'] = '%s' % (status_dict['exposure_val_str'])
        result['#exposure_val_str'] = '%s' % (status_dict['exposure_val_str'])
        result['#autoguider_interval_val'] = '%s' % (status_dict['autoguider_interval_val_str'])
        result['#autoguider_interval_val_str'] = '%s' % (status_dict['autoguider_interval_val_str'])

#        Get View on the whole second.
#        if lt2=='0':
#          result['#guider_view'] = send_autoguider_cmd('{get_view}')
        result['#guider_view'] = send_autoguider_cmd('{get_view}')

        result['status'] = "Success"
      elif action == "set_index":
        response = send_scopeserver_cmd('{set_ra_index_pos}')
        result['status'] = "Success"
      elif action == "toggle_pec":
        response = send_scopeserver_cmd('{toggle_pec}')
        result['status'] = "Success"
      elif action == "imgpick":
        pick_x = request.POST.get("pick_x")
        pick_y = request.POST.get("pick_y")
        response = send_autoguider_cmd('{set_img_pick %s %s}' % (pick_x, pick_y))
        result['status'] = "Success"
      elif action == "clearpick":
        response = send_autoguider_cmd('{clear_img_pick}')
        result['status'] = "Success"
      elif action == "move1to2":
        response = send_autoguider_cmd('{move1to2}')
        result['status'] = "Success"
      elif action == "set_gamma_val":
        value = request.POST.get("value")
        response = send_autoguider_cmd('{set_gamma_val %s}' % (value))
        result['#gamma_val_str'] = '%s' % (value)
        result['status'] = "Success"
      elif action == "set_bp_val":
        value = request.POST.get("value")
        response = send_autoguider_cmd('{set_bp_val %s}' % (value))
        result['#bp_val_str'] = '%s' % (value)
        result['status'] = "Success"
      elif action == "set_mag_val":
        value = request.POST.get("value")
        response = send_autoguider_cmd('{set_mag_val %s}' % (value))
        result['#mag_val_str'] = '%s' % (value)
        result['status'] = "Success"
      elif action == "set_exposure_val":
        value = request.POST.get("value")
        response = send_autoguider_cmd('{set_exposure_val %s}' % (value))
        result['#exposure_val_str'] = '%s' % (value)
        result['status'] = "Success"
      elif action == "set_autoguider_interval_val":
        value = request.POST.get("value")
        response = send_autoguider_cmd('{set_autoguider_interval_val %s}' % (value))
        result['#autoguider_interval_val_str'] = '%s' % (value)
        result['status'] = "Success"
      elif action == "agimage":
        print("Toggle Imaging")
        response = send_autoguider_cmd('{toggle_imaging}')
        result['status'] = "Success"
      elif action == "agfind":
        print("Find Guide Star")
        response = send_autoguider_cmd('{find_guide_star}')
        result['status'] = "Success"
      elif action == "agcenter":
        print("Center Guide Star")
        response = send_autoguider_cmd('{center_guide_star}')
        result['status'] = "Success"
      elif action == "agguide":
        print("Toggle Guiding")
        response = send_scopeserver_cmd('{toggle_guiding}')
        result['status'] = "Success"
      elif action == "agana":
        print("Toggle Analysis")
        response = send_autoguider_cmd('{toggle_analysis}')
        result['status'] = "Success"
      elif action == "agcorr":
        print("Toggle Guide Correction Plot")
        response = send_autoguider_cmd('{toggle_gcorr}')
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
        import os
        import time
        print("RESETTING GPS BY BUTTON PRESS")
        time.sleep(1)
        os.system('nohup sudo /home/pi/src/scopeserver-git/scopeserver/scopeserver_control_gpsreset.sh &')
      elif action == "ssreset":
        import os
        import time
        print("RESETTING SCOPE SERVER BY BUTTON PRESS")
        time.sleep(1)
        os.system('nohup sudo /home/pi/src/scopeserver-git/scopeserver/scopeserver_control_ssreset.sh &')
      elif action == "ssreboot":
        import subprocess as sp
        import time
        print("REBOOTING BY BUTTON PRESS")
        time.sleep(1)
        sp.check_output(["sudo reboot"],shell=True)
      elif action == "ssshutdown":
        import subprocess as sp
        import time
        print("SHUTTING DOWN BY BUTTON PRESS")
        time.sleep(1)
        sp.check_output(["sudo shutdown now"],shell=True)
      elif action == "agauto":
        print("Toggle Autoguider Autoengage")
        response = send_scopeserver_cmd('{toggle_autoguider_auto}')
        result['status'] = "Success"
      elif action == "agreset":
        print("RESETTING AUTOGUIDER BY BUTTON PRESS")
        response = send_autoguider_ud_cmd('{reset_autoguider}')
        result['status'] = "Success"
      elif action == "agreboot":
        print("REBOOTING AUTOGUIDER BY BUTTON PRESS")
        response = send_autoguider_ud_cmd('{reboot_autoguider}')
        result['status'] = "Success"
      elif action == "agshutdown":
        print("SHUTTING DOWN AUTOGUIDER BY BUTTON PRESS")
        response = send_autoguider_ud_cmd('{shutdown_autoguider}')
        result['status'] = "Success"
      elif action == "wifi":
        import os
        import time
        print("CONNECTING TO WiFi NETWORK")
        time.sleep(1)
        os.system('nohup sudo /home/pi/src/scopeserver-git/scopeserver/scopeserver_force_autohotspot.sh &')
    except Exception as e:
      result['status'] = "Failure"
      result['msg'] = str(e)
    finally:
      return(JsonResponse(result))
  elif request.method == "POST":
    pass


#  return(render(request, "server/control.html", locals()))
#  return(render(request, "server/control.html", context))
#  return(render(request, "server/hamburger_body.html", {}))
  return(render(request, "server/control.html", {}))

