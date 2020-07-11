from django.shortcuts import render
from django.http import JsonResponse
from pdb import set_trace as debug

# Create your views here.

import os
import sys
import socket
import subprocess
import glob
import time
import datetime
from io import BytesIO
import base64
from PIL import Image
import random
import traceback


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

  scopeserver_host = get_ip()
  scopeserver_port = 54030
  host_dict = {}
  host_dict['10.0.1.20'] = '10.0.1.23'
  host_dict['192.168.50.5'] = '192.168.50.10'
  autoguider_host = host_dict[scopeserver_host]
  autoguider_port = 54040

  # Create a socket (SOCK_STREAM means a TCP socket)
  scope_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  try:
      # Connect to server and send data
      scope_socket.connect((scopeserver_host, scopeserver_port))
      scope_socket.sendall(cmd.encode('utf-8'))

      # Receive data from the server and shut down
      response = scope_socket.recv(1024).decode('utf-8')
  except:
      tracebackStr = traceback.format_exc()
      raise(Exception("ScopeServer not found at {0}:{1}\n\n{2}".format(HOST, PORT, tracebackStr)))  #This is new. 
  finally:
      scope_socket.close()
#  print("Sent:     {}".format(cmd))
#  print("Received: {}".format(response))
  return response #Response will not exist if there is an exception. 


# Send a command to scope_server socket and receive response
def send_autoguider_cmd(cmd):

  scopeserver_host = get_ip()
  scopeserver_port = 54030
  host_dict = {}
  host_dict['10.0.1.20'] = '10.0.1.23'
  host_dict['192.168.50.5'] = '192.168.50.10'
  autoguider_host = host_dict[scopeserver_host]
  autoguider_port = 54040
  buf_size = int(2**16)

  # Create a socket (SOCK_STREAM means a TCP socket)
  guider_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  response = ''
  try:
    # Connect to server and send data
    guider_socket.settimeout(0.5)
    guider_socket.connect((autoguider_host, autoguider_port))
    guider_socket.settimeout(None)
    guider_socket.sendall(cmd.encode('utf-8'))

    # Receive data from the server and shut down
    response = guider_socket.recv(buf_size).decode('utf-8')
    if cmd == '{get_view}':
      length = int(response) 
      response = b''
      while len(response) < length:
        # doing it in batches is generally better than trying
        # to do it all in one go, so I believe.
        to_read = length - len(response)
        response += guider_socket.recv(buf_size if to_read > buf_size else to_read)
      response = response.decode('utf-8')
  except:
    pass
  finally:
    guider_socket.close()
  return response  #Response will be None if there is an exception. 


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

      elif slew == "slew_track":
          print("Toggle Guide Mode")
          response = send_scopeserver_cmd('{toggle_slew_track}')

      result['status'] = "Success"
      return(JsonResponse(result))
    '''

    action = request.POST.get("action", "")
    try:
      if action == "serverStatus":
        scope_status = send_scopeserver_cmd('{get_status}')
        status_dict = eval(scope_status)
        result['#gps_location_lat'] = '%.5f' % (status_dict['site_latitude'])
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
        result['#meridian_mode'] = 'mflp:  %d' % (status_dict['meridian_mode'])
        result['#scope_position_dec'] = '%s (%d)' % (status_dict['dec_angle'], status_dict['dec_pos'])
        result['#scope_position_ra'] = '%s (%d)' % (status_dict['ra_time'], status_dict['ra_pos'])
        result['#target_position_dec'] = '%s (%d)' % (status_dict['target_dec_angle'], status_dict['target_dec_pos'])
        result['#target_position_ra'] = '%s (%d)' % (status_dict['target_ra_time'], status_dict['target_ra_pos'])
        result['#motor_current_dec'] = '%d' % (status_dict['motor_current_dec'])
        result['#motor_current_ra'] = '%d' % (status_dict['motor_current_ra'])
        result['#pos_error_dec'] = '%d' % (status_dict['pos_error_dec'])
        result['#pos_error_ra'] = '%d' % (status_dict['pos_error_ra'])
        result['#autoguider_connected'] = '%s' % (status_dict['autoguider_connected'])
        result['#autoguider_status'] = '%s' % (status_dict['autoguider_status'])
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
        response = send_autoguider_cmd('{toggle_guiding}')
        result['status'] = "Success"
      elif action == "agana":
        print("Toggle Analysis")
        response = send_autoguider_cmd('{toggle_analysis}')
        result['status'] = "Success"
      elif action == "guiderView":
        result['#guider_view'] = send_autoguider_cmd('{get_view}')
        result['status'] = "Success"
        ''' 
        my_path = os.path.split(os.path.realpath(__file__))[0]
        img_path = os.path.join(my_path,'autoguider_images')
        img_fns = sorted(glob.glob(img_path + '/*'))
        img_fn = random.choice(img_fns)
        img_buf = BytesIO()
        Image.open(img_fn).resize((400,300)).convert('L').save(img_buf,"JPEG")
        result['#guider_view'] = 'data:image/jpg;base64,' + base64.b64encode(img_buf.getvalue()).decode('utf-8')
        result['status'] = "Success"
        ''' 
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
      elif action == "ssreboot":
        print("REBOOTING BY BUTTON PRESS")
        time.sleep(1)
        subprocess.check_output(["sudo reboot"],shell=True)
      elif action == "ssshutdown":
        print("SHUTTING DOWN BY BUTTON PRESS")
        time.sleep(1)
        subprocess.check_output(["sudo shutdown now"],shell=True)
      elif action == "agauto":
        print("Toggle Autoguider Autoengage")
        response = send_scopeserver_cmd('{toggle_autoguider_auto}')
        result['status'] = "Success"
      elif action == "agreboot":
        print("REBOOTING AUTOGUIDER BY BUTTON PRESS")
        response = send_scopeserver_cmd('{reboot_autoguider}')
        result['status'] = "Success"
      elif action == "agshutdown":
        print("SHUTTING DOWN AUTOGUIDER BY BUTTON PRESS")
        response = send_scopeserver_cmd('{shutdown_autoguider}')
        result['status'] = "Success"

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

  '''
  my_path = os.path.split(os.path.realpath(__file__))[0]
  img_path = os.path.join(my_path,'autoguider_images')
  img_fns = sorted(glob.glob(img_path + '/*'))
  img_fn = random.choice(img_fns)

  context = {}
  img_in_memory = BytesIO()
#  img_fn = os.path.join(my_path,'autoguider_images/image_seq_0000.1580619295.3932567.jpg')
  img = Image.open(img_fn)
  img = img.resize((400,300))
  img.save(img_in_memory, format="JPEG")
  context['image'] = base64.b64encode(img_in_memory.getvalue())
  '''

#  return(render(request, "server/control.html", locals()))
#  return(render(request, "server/control.html", context))
  return(render(request, "server/control.html", {}))

