import serial
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



class NmcModule():

  def __init__(self, nmc_net, addr, name):
    self.nmc_net = nmc_net
    self.addr = None
    self.name = name
    self.type = None
    self.len_status = 1 + 1
    self.status_bytes = None
    self.status_dict = None
    self.resp = None
    self.checksum_error = None
    self.SetAddr(0x00, addr, 0xFF)


  def SetAddr(self, curr_addr, new_addr, group):
    cmd = bytearray.fromhex('%02x %02x %02x %02x' % (curr_addr, 0x21, new_addr, group))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('SetAddr response ', self.resp, 'checksum_error ', self.checksum_error)
    if self.checksum_error:
      self.addr = curr_addr
      sys.stderr.write('Error setting module address\n')
    else:
      self.addr = new_addr
    return (self.checksum_error)
  

  def ConfigReset(self):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x1F, 0x00))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ConfigReset response ', self.resp)


  def DefineStatusData(self, status_bits):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x12, status_bits))
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
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('DefineStatusData response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error defining status\n')
    if self.checksum_error == 2:
      self.len_status = 1 + 1
    return (self.resp, self.checksum_error)


  def ReadFullStatus(self):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x13, 0xFF))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 17 + self.len_status)
    if self.checksum_error:
      print('ReadFullStatus response ', self.resp)
      sys.stderr.write('Error getting full status\n')
      return (None)

    self.status_dict = {}
    self.status_dict['pos'] = int.from_bytes(self.resp[1:5], 'big', signed = True)
    self.status_dict['cur_sense'] = int.from_bytes(self.resp[5:6], 'big', signed = False)
    self.status_dict['vel'] = int.from_bytes(self.resp[6:8], 'big', signed = True)
    self.status_dict['aux_status'] = int.from_bytes(self.resp[8:9], 'big', signed = False)
    self.status_dict['home_pos'] = int.from_bytes(self.resp[9:13], 'big', signed = True)
    self.status_dict['device_type'] = nmc_device_dict[int.from_bytes(self.resp[13:14], 'big', signed = False)]
    self.status_dict['device_version'] = int.from_bytes(self.resp[14:15], 'big', signed = False)
    self.status_dict['pos_error'] = int.from_bytes(self.resp[15:17], 'big', signed = True)
    self.status_dict['path_pts'] = int.from_bytes(self.resp[17:18], 'big', signed = False)
    return (self.status_dict)


  def NoOp(self):
    cmd = bytearray.fromhex('%02x %02x' % (self.addr, 0x0E))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    if self.checksum_error:
      print('NoOp response ', self.resp)
      sys.stderr.write('Error sending NoOp\n')
    return (self.resp, self.checksum_error)


  def ServoSetPos(self, pos):
    cmd = bytearray.fromhex('%02x %02x %02x %08x' % (self.addr, 0x50, 0x02, pos))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoSetPos response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error setting position\n')
    return (self.resp, self.checksum_error)

  
  def ServoResetPos(self):
    cmd = bytearray.fromhex('%02x %02x' % (self.addr, 0x00))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoResetPos response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error resetting position\n')
    return (self.resp, self.checksum_error)


  def ServoGetPos(self):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x13, 0x01))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 4 + self.len_status)
    if self.checksum_error:
      sys.stderr.write('Error getting position status\n')
      return (None, self.checksum_error)
    pos = int.from_bytes(self.resp[1:5], 'big', signed = True)
    return (pos, self.checksum_error)


  def ServoSetGain(self, Kp, Kd, Ki, IL, OL, CL, EL, SR, DB, SM):
    cmd = bytearray.fromhex('%02x %02x %04x %04x %04x %04x %02x %02x %04x %02x %02x %02x' % (self.addr, 0xF6, Kp, Kd, Ki, IL, OL, CL, EL, SR, DB, SM))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoSetGain response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error setting servo gain\n')
    return (self.resp, self.checksum_error)


  def ServoStopMotor(self):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x17, 0x03))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoStopMotor response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error in stop motor\n')
    return (self.resp, self.checksum_error)


  def ServoStopAbrupt(self):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x17, 0x05))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoStopAbrupt response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error in abrupt stop\n')
    return (self.resp, self.checksum_error)


  def ServoStopSmooth(self):
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x17, 0x09))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoStopSmooth response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error in smooth stop\n')
    return (self.resp, self.checksum_error)


  def ServoStopHere(self, pos):
    cmd = bytearray.fromhex('%02x %02x %02x %08x' % (self.addr, 0x57, 0x11, pos))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoStopHere response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error in stop here\n')
    return (self.resp, self.checksum_error)


  def ServoLoadTraj(self, mode, pos, vel, acc, pwm):
    n = 1 + ((mode & LOAD_POS) > 0)*4 + ((mode & LOAD_VEL) > 0)*4 + ((mode & LOAD_ACC) > 0)*4 + ((mode & LOAD_PWM) > 0)*1 
    cmd_p1 = bytearray.fromhex('%02x %02x %02x' % (self.addr, 16*n+ 0x04, mode))
    cmd_p2 = b''
    if (mode & LOAD_POS):
      cmd_p2 = cmd_p2 + bytearray.fromhex('%08x' % (pos))
    if (mode & LOAD_VEL):
      cmd_p2 = cmd_p2 + bytearray.fromhex('%08x' % (vel))
    if (mode & LOAD_ACC):
      cmd_p2 = cmd_p2 + bytearray.fromhex('%08x' % (acc))
    if (mode & LOAD_PWM):
      cmd_p2 = cmd_p2 + bytearray.fromhex('%02x' % (pwm))
    cmd = cmd_p1 + cmd_p2
#    print('Sending Trajectory Command: %s' % (cmd))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoLoadTraj response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error in load trajectory\n')
    return (self.resp, self.checksum_error)


  def ServoIOControl(self, limit_mode = False, output_mode = PH3_MODE, fast_path = False, step_dir_mode = False):
    if step_dir_mode:
      limit_mode = False
    control_byte = 0x00 | limit_mode | output_mode | step_dir_mode
    cmd = bytearray.fromhex('%02x %02x %02x' % (self.addr, 0x18, control_byte))
    self.resp, self.checksum_error = self.nmc_net.send_cmd(cmd, 0 + self.len_status)
    print('ServoIOControl response ', self.resp)
    if self.checksum_error:
      sys.stderr.write('Error setting servo I/O control\n')
    return (self.resp, self.checksum_error)



class NmcNet():

  def __init__(self):
    self.port = None
    self.num_modules = 0
    self.modules = {}

  def Initialize(self, module_names = None, port = '/dev/ttyUSB0', baudrate=19200, timeout = 0.02, max_tries = 5):

    # Starting from an unknown state, reset NMC network back to power-up state 
    # Scan baud rates and send simple reset command
    print('Resetting NMC Network to Power-up State...')
    for baud in nmc_baud_rates:
      self.port = serial.Serial(port, baudrate=baud, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout = timeout)
      self.port.write(bytes(20))
      time.sleep(0.002)
      self.port.flushInput()
      self.SimpleReset()
      time.sleep(0.002)
      self.port.close()
    print('NMC Network Now Reset to Power-up State')

    # Now initialize the NMC newtork starting from the power-up state
    print('')
    print('Initializing NMC Network...')
    self.port = serial.Serial(port, baudrate=19200, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout = timeout)

    num_modules_expected = len(module_names)
    self.num_modules = 0
    num_tries = 0
    while (self.num_modules != num_modules_expected) & (num_tries < max_tries):
      # Clear nmc bus
      self.port.write(bytes(20))
      time.sleep(0.002)
      self.port.flushInput()

      self.SimpleReset()

      addr = 1
      for module_name in module_names:
        print('Setting up module:  %s' % (module_name))
        mod = NmcModule(self,addr,module_name)
        if mod.addr:
          addr += 1
          self.modules[mod.name] = mod
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
      print('Successfully Initialized NMC Network in %d Tries' % (num_tries))
    else:
      print('')
      print('Failed to Initialize NMC Network in %d Tries' % (num_tries))

    print('')
    print('Found %d NMC Modules of %d Expected' % (self.num_modules, num_modules_expected))

    print('')
    print('Communicating with NMC Network at Baudrate: %d' % (self.port.getBaudrate()))

    for module_name in sorted(self.modules.keys()):
      mod = self.modules[module_name]
      print('')
      print('Sending NoOp to Check Status of NMC module %d: ' % (mod.addr))
      resp, checksum_error = mod.NoOp()
      print('NoOp response ', resp)
      print('')
      print('Full Status of NMC module %s at addr %d: ' % (mod.name,mod.addr))
      status_dict = mod.ReadFullStatus()
      if status_dict:
        print('  pos:             %d' % (status_dict['pos']))
        print('  cur_sense:       %d' % (status_dict['cur_sense']))
        print('  vel:             %d' % (status_dict['vel']))
        print('  aux_status:      %d' % (status_dict['aux_status']))
        print('  home_pos:        %d' % (status_dict['home_pos']))
        print('  device_type:     %s' % (status_dict['device_type']))
        print('  device_version:  %d' % (status_dict['device_version']))
        print('  pos_error:       %d' % (status_dict['pos_error']))
        print('  path_pts:        %d' % (status_dict['path_pts']))

    print('')

    self.SetBaud(baudrate)

    print('')
    print('Communicating with NMC Network at Baudrate: %d' % (self.port.getBaudrate()))

    for module_name in sorted(self.modules.keys()):
      mod = self.modules[module_name]
      print('')
      print('Full Status of NMC module %s at addr %d: ' % (mod.name,mod.addr))
      status_dict = mod.ReadFullStatus()
      if status_dict:
        print('  pos:             %d' % (status_dict['pos']))
        print('  cur_sense:       %d' % (status_dict['cur_sense']))
        print('  vel:             %d' % (status_dict['vel']))
        print('  aux_status:      %d' % (status_dict['aux_status']))
        print('  home_pos:        %d' % (status_dict['home_pos']))
        print('  device_type:     %s' % (status_dict['device_type']))
        print('  device_version:  %d' % (status_dict['device_version']))
        print('  pos_error:       %d' % (status_dict['pos_error']))
        print('  path_pts:        %d' % (status_dict['path_pts']))


    print('')


  def SimpleReset(self):
    cmd = bytearray.fromhex('%02x %02x' % (0xFF, 0x0F))
    self.send_cmd(cmd, 2)
    print('SimpleReset')


  def Shutdown(self):
    print('Shutting Down NMC Network')
    self.SimpleReset()
    time.sleep(0.002)
    self.port.close()


  def SetBaud(self, baudrate):
    print('Switching to Baudrate: %d' % (baudrate))
    cmd = bytearray.fromhex('%02x %02x %02x' % (0xFF, 0x1A, nmc_baud_code[baudrate]))
    full_cmd = bytes([0xAA]) + cmd + bytes([self.checksum_8(cmd)])
    self.port.write(full_cmd)
    time.sleep(0.1)
    self.port.setBaudrate(baudrate)
    for module_name in sorted(self.modules.keys()):
      mod = self.modules[module_name]
      print('')
      print('Sending NoOp to Check Status of NMC module %s at addr %d: ' % (mod.name, mod.addr))
      resp, checksum_error = mod.NoOp()
      print('NoOp response ', resp)



  def send_cmd(self, cmd, len_response):
    full_cmd = bytes([0xAA]) + cmd + bytes([self.checksum_8(cmd)])
    self.port.write(full_cmd)
    resp = self.port.read(len_response)

    checksum_error = self.checksum_check(resp)
    if checksum_error == 2:
      module = int.from_bytes(cmd[1:2], 'big', signed = False)
      sys.stderr.write('Host-to-NMC checksum error reported by module 0x%02x\n' % (module))
    return (resp, checksum_error)


  # Check validity of command-response communication
  def checksum_check(self, response):
    # check if response contains data
    if len(response):
      # if so then check if received response is valid
      if self.checksum_8(response[:-1]) != response[-1]:
        return 1
      # if so then check if previously sent command was received correctly
      return (response[0] & CKSUM_ERR)
    else:
      return 1


  def checksum_8(self, bytes_arg):
    return sum(bytes_arg)%256


