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
import os
import queue
import subprocess as sp

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# Signal handler for shutdown
def signal_handler(signal, frame):
  global fd, old_settings
  try:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  except:
    pass
  sys.stderr.write('\r\nAutoGuider UpDown Server caught signal %d\r\n' % (signal))
  sys.stderr.write('AutoGuider UpDown Server shutting down.\r\n')

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


class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):
  def handle(self):
    try:
      while agud.server_running:
      # Receive data from client
        data = self.request.recv(1024)
#        sys.stderr.write('Server received:  "%s"\r\n' % (data.decode('ascii')))
        if data:
          # Process client command
          cmd = data.decode('utf-8').strip()
          if cmd:
            if cmd[0] == '{':
              cmd = cmd[1:-1]
              response = agud.process_autoguider_ud_cmd(cmd)
            if response:
              # Send response to client
              response = response.encode('utf-8')
#              sys.stderr.write( 'Server responding:  %s\r\n' % (response) )
#              sys.stderr.write( 'Server sending:  %d bytes\r\n' % (len(response)) )
              self.request.sendall(response)
        else:
          return
    except:
      sys.stderr.write('Exception in Autoguider UpDown Server TCP Command Processor.\r\n')
      pass
#    except ConnectionResetError:
#      sys.stderr.write('Connection reset by peer.\r\n')
#      pass


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
  allow_reuse_address = True
  daemon_threads = True
  timeout = None
  request_queue_size = 20


class autoguider_updown:

  def __init__(self, server_address, port):
    self.input_running = False
    self.server_running = False
    self.autoguider_ud_q = queue.Queue(maxsize=0)
    self.autoguider_ud_running = False

    self.autoguider_ud_host = server_address
    self.autoguider_ud_port = port
    host_dict = {}
    host_dict['10.0.1.14'] = '10.0.1.13'
    host_dict['10.0.1.23'] = '10.0.1.20'
    host_dict['10.0.1.24'] = '10.0.1.23'
    host_dict['192.168.50.10'] = '192.168.50.5'
    self.scopeserver_host = host_dict[self.autoguider_ud_host]
    self.scopeserver_port = 54030

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGHUP, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGQUIT, signal_handler)


  def get_status(self):
    status_str = '%s %d' % (self.status, self.img_count)
    return status_str


  # Start server thread
  def server_start(self):
    self.autoguider_ud_control_start()
    self.server_running = True

    self.threaded_server = ThreadedTCPServer((self.autoguider_ud_host, self.autoguider_ud_port), ThreadedTCPRequestHandler)

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    self.server_thread = threading.Thread(target=self.threaded_server.serve_forever)
    # Exit the server thread when the main thread terminates
    self.server_thread.daemon = True
    self.server_thread.start()

    self.server_thread.join()


  # Stop server thread
  def server_stop(self):
    if self.server_running:
      self.server_running=False
      sys.stderr.write('AutoGuider UpDown Server Shutting Down.\r\n')
      self.threaded_server.shutdown()


  # Start thread for autoguider control
  def autoguider_ud_control_start(self):
    self.thread_autoguider_ud_control = threading.Thread(target=self.autoguider_ud_control)
    self.thread_autoguider_ud_control.setDaemon(1)
    self.thread_autoguider_ud_control.start()


  def autoguider_ud_control(self):
    sys.stderr.write('\r\nautoguider_ud_control: thread starting...\r\n')
    while True:
      if self.autoguider_ud_running:
        # if still in progress get next command but do not wait if queue is empty
        try:
          autoguider_ud_cmd = self.autoguider_ud_q.get_nowait()
        except queue.Empty:
          autoguider_ud_cmd = ('autoguider_ud_continue', None)
      else:
        # if not still in progress wait to receive next command
#        sys.stderr.write('autoguiding_ud_control: idle, waiting for command\r\n')
        autoguider_ud_cmd = self.autoguider_ud_q.get()

      cmd = autoguider_ud_cmd[0]
      cmd_arg = autoguider_ud_cmd[1]

      if cmd == 'reset_autoguider':
        print('Resetting Autoguider...')
        time.sleep(1)
        os.system('sudo systemctl restart autoguider_control.service')

      elif cmd == 'reboot_autoguider':
        time.sleep(1)
        os.system('reboot')

      elif cmd == 'shutdown_autoguider':
        time.sleep(1)
        os.system('shutdown now')

      elif cmd == 'autoguider_ud_continue':
        pass


      if not cmd == 'autoguider_ud_continue':
        self.autoguider_ud_q.task_done()


  # Process BlueShift Telescope Autoguider UpDown Server Protocol Commands
  def process_autoguider_ud_cmd(self,cmd):

    response = None

    if cmd == 'reset_autoguider':
      response = 'reset_autoguider'
      self.autoguider_ud_q.put(('reset_autoguider', None))

    elif cmd == 'reboot_autoguider':
      response = 'reboot_autoguider'
      self.autoguider_ud_q.put(('reboot_autoguider', None))

    elif cmd == 'shutdown_autoguider':
      response = 'shutdown_autoguider'
      self.autoguider_ud_q.put(('shutdown_autoguider', None))

    else:
      pass

    return response



if (__name__ == '__main__'):

  pid = os.getpid()
  try:
    pidfile = open('/var/run/autoguider_updown_server.pid','w')
    pidfile.write('%d\n' % (pid))
    pidfile.close()
  except:
    pass

  server_address = get_ip()
  port = 54050

  sys.stderr.write('\r\nStarting AutoGuider UpDown Server at %s:%d\r\n' % (server_address, port))

  agud = autoguider_updown(server_address, port)
  agud.server_start()

