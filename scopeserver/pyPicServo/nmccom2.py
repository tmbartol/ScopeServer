import serial
import pylibftdi
import time
import sys


# Baud rates for NMC network
nmc_baud_rates = [9600, 19200, 57600, 115200, 230400]
nmc_baud_code = {}
nmc_baud_code[9600] =  127
nmc_baud_code[19200] =  64
nmc_baud_code[57600] =  21
nmc_baud_code[115200] = 10
nmc_baud_code[230400] =  5

# Device codes on NMC network
nmc_device_dict = {}
nmc_device_dict[0] = 'PIC-SERVO SC'
nmc_device_dict[2] = 'PIC-I/O'
nmc_device_dict[3] = 'PIC-STEP'

# Status Byte bit masks
MOVE_DONE =    0x01
CKSUM_ERR =    0x02
PWR_ON =       0x04
POS_ERR =      0x08
LIMIT1 =       0x10
LIMIT2 =       0x20
HOME_IN_PROG = 0x40

# Status Data bit masks
SEND_POS =       0x01
SEND_CUR_SENSE = 0x02
SEND_VEL =       0x04
SEND_AUX =       0x08
SEND_HOME_POS =  0x10
SEND_DEV_TYPE =  0x20
SEND_POS_ERR =   0x40
SEND_PATH_PTS =  0x80

# Servo Module LOAD_TRAJ control byte bit definitions:
LOAD_POS =          0x01  # +4 bytes
LOAD_VEL =          0x02  # +4 bytes
LOAD_ACC =          0x04  # +4 bytes
LOAD_PWM =          0x08  # +1 byte
ENABLE_SERVO =      0x10  # 1 = servo mode, 0 = PWM mode
VEL_MODE =          0x20  # 1 = velocity mode, 0 = trap. position mode
REVERSE =           0x40  # 1 = command neg. PWM or vel, 0 = positive
MOVE_REL =          0x40  # 1 = move relative, 0 = move absolute
START_NOW =         0x80  # 1 = start now, 0 = wait for START_MOVE command


# Servo I/O Control bit flags
# limit mode flags:
LIMIT_OFF =       0x04
LIMIT_STOP =      0x08

# output mode flags:
PWM_DIR_MODE =    0x00
PH3_MODE =        0x10
ANTIPHASE_MODE =  0x20

# Fast Path mode flag:
FASTPATH_MODE =   0x40

# Step & Direction flag:
STEP_DIR_MODE =   0x80



# Compute 8-bit checksum of an array of bytes
def checksum_8(bytes_arg):
  return sum(bytes_arg)%256



class NmcModule():

  def __init__(self, name, nmc_net, addr):
    self.name = name
    self.nmc_net = nmc_net
    self.addr = 0
    self.group = None
    self.device_type = None
    self.device_version = None
    self.len_status = 1 + 1
    self.status_data = None
    self.status_dict = None
    self.response = None
    self.checksum_error = None
    self.send_errors = 0
    self.receive_errors = 0
    self.cmd_msg = ''
    self.err_msg = ''
    self.verbosity = 2 # 0: quiet,  1: errors only,  2: error and cmd messages
    self.SetAddr(addr)


  def PrintMsg(self):
    if self.verbosity:
      if self.err_msg:
        sys.stderr.write('>>> NMC Error: Module %s at addr %d:\n    >>> %s' % (self.name, self.addr, self.err_msg))
        self.err_msg = ''
      if self.verbosity == 2:
        if self.cmd_msg:
          sys.stdout.write('CMD Status: Module %s at addr %d:\n    %s' % (self.name, self.addr, self.cmd_msg))
          self.cmd_msg = ''


  def SetAddr(self, new_addr, group=0xFF):
    cmd = bytes([self.addr, 0x21, new_addr, group])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('SetAddr response: %s  checksum_error: %a\n' % (self.response, self.checksum_error))
    if self.checksum_error:
      self.err_msg = 'SetAddr: Error setting module address\n'
    else:
      self.addr = new_addr
      self.group = group
    self.PrintMsg()
  

  def ConfigReset(self):
    cmd = bytes([self.addr, 0x1F, 0x00])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ConfigReset response: %s\n' % (self.response))
    self.PrintMsg()


  def DefineStatusData(self, status_bits):
    cmd = bytes([self.addr, 0x12, status_bits])
    self.len_status = 1 + 1
    extra_bytes = 0
    if status_bits & SEND_POS:
      extra_bytes += 4
    if status_bits & SEND_CUR_SENSE:
      extra_bytes += 1
    if status_bits & SEND_VEL:
      extra_bytes += 2
    if status_bits & SEND_AUX:
      extra_bytes += 1
    if status_bits & SEND_HOME_POS:
      extra_bytes += 4
    if status_bits & SEND_DEV_TYPE:
      extra_bytes += 2
    if status_bits & SEND_POS_ERR:
      extra_bytes += 2
    if status_bits & SEND_PATH_PTS:
      extra_bytes += 1
    self.len_status += extra_bytes
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('DefineStatusData response: %s  Status Bits: 0x%02x\n' % (self.response, status_bits))
    if self.checksum_error:
      self.err_msg = ('DefineStatusData: Error defining status\n')
    if self.checksum_error == 2:
      self.len_status = 1 + 1
    self.PrintMsg()


  def ReadFullStatus(self):
    cmd = bytes([self.addr, 0x13, 0xFF])
    self.SendCmd(cmd, 17 + self.len_status)
    self.cmd_msg = ('ReadFullStatus response: %s\n' % (self.response))
    if self.checksum_error:
      self.err_msg = ('ReadFullStatus: Error reading full status\n')
      self.status_dict = None
      return
    self.PrintMsg()

    self.status_dict = {}
    self.status_dict['pos'] = int.from_bytes(self.response[1:5], 'big', signed = True)
    self.status_dict['cur_sense'] = int.from_bytes(self.response[5:6], 'big', signed = False)
    self.status_dict['vel'] = int.from_bytes(self.response[6:8], 'big', signed = True)
    self.status_dict['aux_status'] = int.from_bytes(self.response[8:9], 'big', signed = False)
    self.status_dict['home_pos'] = int.from_bytes(self.response[9:13], 'big', signed = True)
    self.status_dict['device_type'] = nmc_device_dict[int.from_bytes(self.response[13:14], 'big', signed = False)]
    self.device_type = self.status_dict['device_type']
    self.status_dict['device_version'] = int.from_bytes(self.response[14:15], 'big', signed = False)
    self.device_version = self.status_dict['device_version']
    self.status_dict['pos_error'] = int.from_bytes(self.response[15:17], 'big', signed = True)
    self.status_dict['path_pts'] = int.from_bytes(self.response[17:18], 'big', signed = False)


  def NoOp(self):
    cmd = bytes([self.addr, 0x0E])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('NoOp response: %s\n' % (self.response))
    if self.checksum_error:
      self.err_msg = ('NoOp: Error sending NoOp\n')
    self.PrintMsg()


  def ServoSetPos(self, pos):
    cmd = bytes([self.addr, 0x50, 0x02])
    cmd = cmd + bytes.fromhex('%08x' % (pos))
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ServoSetPos response: %s  Pos: %d\n' % (self.response, pos))
    if self.checksum_error:
      self.err_msg = ('ServoSetPos: Error setting position\n')
    self.PrintMsg()

  
  def ServoResetPos(self):
    cmd = bytes([self.addr, 0x00])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ServoResetPos response: %s  Pos: %d\n' % (self.response, 0))
    if self.checksum_error:
      self.err_msg = ('ServoResetPos: Error resetting position\n')
    self.PrintMsg()


  def ServoGetPos(self):
    cmd = bytes([self.addr, 0x13, 0x01])
    self.SendCmd(cmd, 4 + self.len_status)
    if self.checksum_error:
      self.err_msg = ('ServoGetPos: Error getting position\n')
      pos = None
    else:
      pos = int.from_bytes(self.response[1:5], 'big', signed = True)
    self.cmd_msg = ('ServoGetPos response: %s  Pos: %a\n' % (self.response, pos))
    self.PrintMsg()
    return (pos)


  def ServoSetGain(self, Kp, Kd, Ki, IL, OL, CL, EL, SR, DB, SM):
    cmd = bytes.fromhex('%02x %02x %04x %04x %04x %04x %02x %02x %04x %02x %02x %02x' % (self.addr, 0xF6, Kp, Kd, Ki, IL, OL, CL, EL, SR, DB, SM))
    self.SendCmd(cmd, 0 + self.len_status)
    if self.verbosity == 2:
      self.cmd_msg = ('ServoSetGain response: %s\n' % (self.response))
      self.cmd_msg += ('    Kp = %d\n' % (Kp))
      self.cmd_msg += ('    Kd = %d\n' % (Kd))
      self.cmd_msg += ('    Ki = %d\n' % (Ki))
      self.cmd_msg += ('    IL = %d\n' % (IL))
      self.cmd_msg += ('    OL = %d\n' % (OL))
      self.cmd_msg += ('    CL = %d\n' % (CL))
      self.cmd_msg += ('    EL = %d\n' % (EL))
      self.cmd_msg += ('    SR = %d\n' % (SR))
      self.cmd_msg += ('    DB = %d\n' % (DB))
      self.cmd_msg += ('    SM = %d\n' % (SM))
    if self.checksum_error:
      self.err_msg = ('ServoSetGain: Error setting servo gain\n')
    self.PrintMsg()


  def ServoStopMotor(self):
    cmd = bytes([self.addr, 0x17, 0x03])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ServoStopMotor response: %s\n' % (self.response))
    if self.checksum_error:
      self.err_msg = ('ServoStopMotor: Error in stop motor\n')
    self.PrintMsg()


  def ServoStopAbrupt(self):
    cmd = bytes([self.addr, 0x17, 0x05])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ServoStopAbrupt response: %s\n' % (self.response))
    if self.checksum_error:
      self.err_msg = ('ServoStopAbrupt: Error in abrupt stop\n')
    self.PrintMsg()


  def ServoStopSmooth(self):
    cmd = bytes([self.addr, 0x17, 0x09])
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ServoStopSmooth response: %s\n' % (self.response))
    if self.checksum_error:
      self.err_msg = ('ServoStopSmooth: Error in smooth stop\n')
    self.PrintMsg()


  def ServoStopHere(self, pos):
    cmd = bytes([self.addr, 0x57, 0x11])
    cmd = cmd + bytes.fromhex('%08x' % (pos))
    self.SendCmd(cmd, 0 + self.len_status)
    self.cmd_msg = ('ServoStopHere response: %s  Pos: %d\n' % (self.response, pos))
    if self.checksum_error:
      self.err_msg = ('ServoStopHere: Error in stop here\n')
    self.PrintMsg()


  def ServoLoadTraj(self, mode, pos=None, vel=None, acc=None, pwm=None):
    n = 1 + ((mode & LOAD_POS) > 0)*4 + ((mode & LOAD_VEL) > 0)*4 + ((mode & LOAD_ACC) > 0)*4 + ((mode & LOAD_PWM) > 0)*1 
    cmd_p1 = bytes([self.addr, 16*n + 0x04, mode])
    cmd_p2 = bytes(0)
    if (mode & LOAD_POS):
      cmd_p2 = cmd_p2 + bytes.fromhex('%08x' % (pos))
    if (mode & LOAD_VEL):
      cmd_p2 = cmd_p2 + bytes.fromhex('%08x' % (vel))
    if (mode & LOAD_ACC):
      cmd_p2 = cmd_p2 + bytes.fromhex('%08x' % (acc))
    if (mode & LOAD_PWM):
      cmd_p2 = cmd_p2 + bytes([pwm])
    cmd = cmd_p1 + cmd_p2
#    print('Sending Trajectory Command: %s' % (cmd))
    self.SendCmd(cmd, 0 + self.len_status)
    if self.verbosity == 2:
      self.cmd_msg = ('ServoLoadTraj response: %s\n' % (self.response))
      self.cmd_msg += ('    mode: 0x%02x\n' % (mode))
      self.cmd_msg += ('     pos: %a\n'     % (pos))
      self.cmd_msg += ('     vel: %a\n'     % (vel))
      self.cmd_msg += ('     acc: %a\n'     % (acc))
      self.cmd_msg += ('     pwm: %a\n'     % (pwm))
    if self.checksum_error:
      self.err_msg = ('ServoLoadTraj: Error in load trajectory\n')
    self.PrintMsg()


  def ServoIOControl(self, limit_mode = False, output_mode = PH3_MODE, fast_path = False, step_dir_mode = False):
    if step_dir_mode:
      limit_mode = False
    control_byte = 0x00 | limit_mode | output_mode | step_dir_mode
    cmd = bytes([self.addr, 0x18, control_byte])
    self.SendCmd(cmd, 0 + self.len_status)
    if self.verbosity == 2:
      self.cmd_msg = ('ServoIOControl response: %s\n' % (self.response))
      self.cmd_msg += ('       limit_mode: %a\n' % (limit_mode))
      self.cmd_msg += ('      output_mode: %a\n' % (output_mode))
      self.cmd_msg += ('        fast_path: %a\n' % (fast_path))
      self.cmd_msg += ('    step_dir_mode: %a\n' % (step_dir_mode))
    if self.checksum_error:
      self.err_msg = ('ServoIOControl: Error setting servo I/O control\n')
    self.PrintMsg()


  def PrintFullStatusReport(self):
    print('')
    print('Full Status of NMC module %s at addr %d: ' % (self.name,self.addr))
    self.ReadFullStatus()
    if self.status_dict:
      print('             pos:  %d' % (self.status_dict['pos']))
      print('       cur_sense:  %d' % (self.status_dict['cur_sense']))
      print('             vel:  %d' % (self.status_dict['vel']))
      print('      aux_status:  %d' % (self.status_dict['aux_status']))
      print('        home_pos:  %d' % (self.status_dict['home_pos']))
      print('     device_type:  %s' % (self.status_dict['device_type']))
      print('  device_version:  %d' % (self.status_dict['device_version']))
      print('       pos_error:  %d' % (self.status_dict['pos_error']))
      print('        path_pts:  %d' % (self.status_dict['path_pts']))
    print('')


  def SendCmd(self, cmd, len_response):
    full_cmd = bytes([0xAA]) + cmd + bytes([checksum_8(cmd)])
    if self.verbosity == 2:
      sys.stdout.write('SendCmd Sending Command: %a\n' % (full_cmd))
    self.nmc_net.port.write(full_cmd)
    self.response = self.nmc_net.port.read(len_response)
    self.CheckResponse()
    if self.checksum_error == 1:
      self.receive_errors += 1
      self.nmc_net.receive_errors += 1
    if self.checksum_error == 2:
      mod_addr = int.from_bytes(cmd[1:2], 'big', signed = False)
      self.send_errors += 1
      self.nmc_net.send_errors += 1
      sys.stderr.write('>>>>>> Host-to-NMC checksum error reported by module %s at addr 0x%02x\n' % (self.name, mod_addr))


  # Check validity of command-response communication
  def CheckResponse(self):
    # check if response contains data
    if len(self.response):
      # if we have a response then check if received response is valid
      if checksum_8(self.response[:-1]) != self.response[-1]:
        self.checksum_error = 1
        return
      # if response OK then check if previously sent command was received correctly
      self.checksum_error = (self.response[0] & CKSUM_ERR)
      return
    else:
      self.checksum_error = 1
      return



class NmcNet():

  def __init__(self):
    self.port = None
    self.num_modules = 0
    self.send_errors = 0
    self.receive_errors = 0
    self.modules = {}


  def Initialize(self, module_names = None, port = '/dev/ttyUSB0', baudrate = 19200, timeout = 0.02, max_tries = 5):


    # Starting from an unknown state, reset NMC network back to power-up state 
    # This is accomplished by scanning baud rates and sending the SimpleReset command
    print('NmcNet: Resetting NMC Network to Power-up State...')
    self.port = pylibftdi.Device()
    self.port.ftdi_fn.ftdi_set_line_property(8, 1, 0)
    self.port.ftdi_fn.ftdi_set_latency_timer(16)
    for baud in nmc_baud_rates:
#      self.port = serial.Serial(port, baudrate=baud, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout = timeout)
      self.port.baudrate = baud
      self.port.write(bytes(20))
      time.sleep(0.002)
#      self.port.flushInput()
      self.port.flush_input()
      self.SimpleReset()
#      self.port.close()
    print('NmcNet: NMC Network Now Reset to Power-up State')

    # Now initialize the NMC newtork starting from the power-up state at 19200 baud
    print('')
    print('NmcNet: Initializing NMC Network...')
#    self.port = serial.Serial(port, baudrate=19200, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout = timeout)
    self.port.baudrate = 19200

    num_modules_expected = len(module_names)
    self.num_modules = 0
    num_tries = 0
    while (self.num_modules != num_modules_expected) & (num_tries < max_tries):
      # Clear NMC bus
      self.port.write(bytes(20))
      time.sleep(0.002)
#      self.port.flushInput()
      self.port.flush_input()
      self.SimpleReset()

      addr = 1
      for module_name in module_names:
        print('NmcNet: Setting up module:  %s' % (module_name))
        mod = NmcModule(module_name, self, addr)
        if mod.addr:
          self.modules[mod.name] = mod
          addr += 1
        else:
          del mod
          break

#      while (not self.NmcSetAddr(0x00, addr, 0xFF)):
#        mod = NmcModule(addr,module_names[addr-1])
#        self.modules[mod.name] = mod
#        addr += 1

      self.num_modules = addr-1
      num_tries += 1

    if self.num_modules == num_modules_expected:
      print('')
      print('NmcNet: Successfully Initialized NMC Network in %d Tries' % (num_tries))
    else:
      print('')
      print('NmcNet: Failed to Initialize NMC Network in %d Tries' % (num_tries))

    print('')
    print('NmcNet: Found %d NMC Modules of %d Expected' % (self.num_modules, num_modules_expected))

    print('')
    print('NmcNet: Communicating with NMC Network at Baudrate: %d' % (self.port.baudrate))

    for module_name in sorted(self.modules.keys()):
      mod = self.modules[module_name]
      print('')
      print('NmcNet: Sending NoOp to Check Status of NMC module %d: ' % (mod.addr))
      mod.NoOp()
      print('NmcNet: NoOp response: %s' % (mod.response))
      mod.PrintFullStatusReport()

    print('')


    if baudrate != 19200:

      self.SetBaud(baudrate)

      print('')
      print('NmcNet: Communicating with NMC Network at Baudrate: %d' % (self.port.baudrate))

      for module_name in sorted(self.modules.keys()):
        mod = self.modules[module_name]
        mod.PrintFullStatusReport()

      print('')

    print('NmcNet Initialize: Send Errors: %d   Receive Errors: %d' % (self.send_errors, self.receive_errors))


  def SimpleReset(self):
    cmd = bytes([0xFF, 0x0F])
    full_cmd = bytes([0xAA]) + cmd + bytes([checksum_8(cmd)])
    self.port.write(full_cmd)
    response = self.port.read(2)
    time.sleep(0.002)
#    self.SendCmd(cmd, 2)
    print('NmcNet: SimpleReset')


  def Shutdown(self):
    print('NmcNet: Shutting Down NMC Network...')
    self.SimpleReset()
    time.sleep(0.002)
    self.port.close()


  def SetBaud(self, baudrate):
    print('NmcNet SetBaud: Switching to Baudrate: %d' % (baudrate))
    # Simultaneously set baudrate of all modules on network
    cmd = bytes([0xFF, 0x1A, nmc_baud_code[baudrate]])
    full_cmd = bytes([0xAA]) + cmd + bytes([checksum_8(cmd)])
    self.port.write(full_cmd)

    # Give modules time to comply with command
    time.sleep(0.1)

    # Set serial port to new baudrate
    self.port.baudrate = baudrate

    # Send NoOp to check status of each NMC module
    for module_name in sorted(self.modules.keys()):
      mod = self.modules[module_name]
      print('')
      print('NmcNet SetBaud: Sending NoOp to Check Status of NMC module %s at addr %d: ' % (mod.name, mod.addr))
      mod.NoOp()
      print('NmcNet SetBaud: NoOp response: %s' % (mod.response))


