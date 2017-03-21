import serial
import time
import sys


class NmcNet():

  def __init__(self):
    self.baud_code = {}
    self.baud_code[9600] =  127
    self.baud_code[19200] =  64
    self.baud_code[57600] =  21
    self.baud_code[115200] = 10
    self.baud_code[230400] =  5

    # Status Byte bit masks
    self.MOVE_DONE =    0x01
    self.CKSUM_ERR =    0x02
    self.PWR_ON =       0x04
    self.POS_ERR =      0x08
    self.LIMIT1 =       0x10
    self.LIMIT2 =       0x20
    self.HOME_IN_PROG = 0x40

    # Status Data bit masks
    self.SEND_POS =       0x01
    self.SEND_CUR_SENSE = 0x02
    self.SEND_VEL =       0x04
    self.SEND_AUX =       0x08
    self.SEND_HOME_POS =  0x10
    self.SEND_DEV_TYPE =  0x20
    self.SEND_POS_ERR =   0x40
    self.SEND_PATH_PTS =  0x80
    
    self.device_dict = {}
    self.device_dict[0] = 'PIC-SERVO SC'
    self.device_dict[2] = 'PIC-I/O'
    self.device_dict[3] = 'PIC-STEP'


  def NmcInit(self, port = '/dev/ttyUSB0', baudrate=19200, timeout = 0.01, num_nmc_expected = 1, max_tries = 5):
    
    self.port = serial.Serial(port, baudrate=baudrate, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout = timeout)

    self.len_status = 1 + 1
    self.num_nmc = 0
    num_tries = 0
    while (self.num_nmc != num_nmc_expected) & (num_tries < max_tries):
      # Clear nmc bus
#      self.port.write(bytes(20))
      self.port.flushOutput()
      self.port.flushInput()
      time.sleep(1.0)

      self.NmcSimpleReset()
      self.NmcSimpleReset()
      addr = 0
      while self.NmcSetAddr(0x00, addr+1, 0xFF):
        addr += 1
      self.num_nmc = addr
      num_tries += 1

    if self.num_nmc == num_nmc_expected:
      print('')
      print('Successfully initialized NMC network in %d tries' % (num_tries))
    else:
      print('')
      print('Failed to initialize NMC network in %d tries' % (num_tries))

    print('')
    print('Found %d NMC modules of %d expected' % (self.num_nmc, num_nmc_expected))

    for addr in range(1,self.num_nmc+1):
      print('')
      print('Status of NMC module %d: ' % (addr))
      status_dict = self.NmcReadFullStatus(addr)
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


  def NmcSetAddr(self, curr_addr, new_addr, group):
    cmd = bytearray.fromhex('%02x %02x %02x %02x' % (curr_addr, 0x21, new_addr, group))
    resp, checksum_error = self.send_cmd(cmd, 0)
    print('NmcSetAddr response ', resp)
    if checksum_error:
      sys.stderr.write('Error setting module address\n')
    return (resp, checksum_error)

  
  def NmcSimpleReset(self):
    cmd = bytearray.fromhex('%02x %02x' % (0xFF, 0x0F))
    resp, checksum_error = self.send_cmd(cmd, 0)
    print('NmcSimpleReset response ', resp)


  def NmcHardReset(self, module):
    cmd = bytearray.fromhex('%02x %02x %02x' % (module, 0x1F, 0x00))
    resp, checksum_error = self.send_cmd(cmd, 0)
    print('NmcHardReset response ', resp)


  def NmcSetBaud(self, baudrate):
    cmd = bytearray.fromhex('%02x %02x' % (0xFF, self.baud_code[baudrate]))
    full_cmd = bytes([0xAA]) + cmd + bytes([self.checksum_8(cmd)])
    self.port.write(full_cmd)
    self.port.baudrate = baudrate
    time.sleep(1.0)
    resp = self.port.read(self.len_status)
    checksum_error = self.checksum_check(resp)
    if checksum_error == 2:
      module = int.from_bytes(cmd[1:2], 'big', signed = False)
      sys.stderr.write('Checksum error reported by module 0x%02x\n' % (module))
    print('NmcSetBaud response ', resp)
    return (resp, checksum_error)


  def NmcDefineStatusData(self, module, status_bits):
    cmd = bytearray.fromhex('%02x %02x %02x' % (module, 0x12, status_bits))
    resp, checksum_error = self.send_cmd(cmd, 0)
    print('NmcDefineStatus response ', resp)
    if checksum_error:
      sys.stderr.write('Error defining status\n')
    if checksum_error != 2:
      extra_bytes = 0
      if status_bits & self.SEND_POS:
        extra_bytes += 4
      if status_bits & self.SEND_CUR_SENSE:
        extra_bytes += 1
      if status_bits & self.SEND_VEL:
        extra_bytes += 2
      if status_bits & self.SEND_AUX:
        extra_bytes += 1
      if status_bits & self.SEND_HOME_POS:
        extra_bytes += 4
      if status_bits & self.SEND_DEV_TYPE:
        extra_bytes += 2
      if status_bits & self.SEND_POS_ERR:
        extra_bytes += 2
      if status_bits & self.SEND_PATH_PTS:
        extra_bytes += 1
      self.len_status += extra_bytes
    return (resp, checksum_error)


  def NmcReadFullStatus(self, module):
    cmd = bytearray.fromhex('%02x %02x %02x' % (module, 0x13, 0xFF))
    resp, checksum_error = self.send_cmd(cmd, 17)
    if checksum_error:
      sys.stderr.write('Error getting status\n')
      return (None, checksum_error)

    status_dict = {}
    status_dict['pos'] = int.from_bytes(resp[1:5], 'big', signed = True)
    status_dict['cur_sense'] = int.from_bytes(resp[5:6], 'big', signed = False)
    status_dict['vel'] = int.from_bytes(resp[6:8], 'big', signed = True)
    status_dict['aux_status'] = int.from_bytes(resp[8:9], 'big', signed = False)
    status_dict['home_pos'] = int.from_bytes(resp[9:13], 'big', signed = True)
    status_dict['device_type'] = self.device_dict[int.from_bytes(resp[13:14], 'big', signed = False)]
    status_dict['device_version'] = int.from_bytes(resp[14:15], 'big', signed = False)
    status_dict['pos_error'] = int.from_bytes(resp[15:17], 'big', signed = True)
    status_dict['path_pts'] = int.from_bytes(resp[17:18], 'big', signed = False)
    return (status_dict, checksum_error)


  def ServoSetPos(self, module, pos):
    cmd = bytearray.fromhex('%02x %02x %02x %08x' % (module, 0x50, 0x02, pos))
    resp, checksum_error = self.send_cmd(cmd, 0)
    print('ServoSetPos response ', resp)
    if checksum_error:
      sys.stderr.write('Error setting position\n')
    return (resp, checksum_error)

  
  def ServoResetPos(self, module):
    cmd = bytearray.fromhex('%02x %02x' % (module, 0x00))
    resp, checksum_error = self.send_cmd(cmd, 0)
    print('ServoResetPos response ', resp)
    if checksum_error:
      sys.stderr.write('Error resetting position\n')
    return (resp, checksum_error)


  def ServoGetPos(self, module):
    cmd = bytearray.fromhex('%02x %02x %02x' % (module, 0x13, 0x01))
    resp, checksum_error = self.send_cmd(cmd, 4)
    if checksum_error:
      sys.stderr.write('Error getting status\n')
      return (None, checksum_error)
    pos = int.from_bytes(resp[1:5], 'big', signed = True)
    return (pos, checksum_error)


  def send_cmd(self, cmd, len_response):
    full_cmd = bytes([0xAA]) + cmd + bytes([self.checksum_8(cmd)])
    self.port.write(full_cmd)
    resp = self.port.read(self.len_status + len_response)
    checksum_error = self.checksum_check(resp)
    if checksum_error == 2:
      module = int.from_bytes(cmd[1:2], 'big', signed = False)
      sys.stderr.write('Checksum error reported by module 0x%02x\n' % (module))
    return (resp, checksum_error)


  # Check validity of command-response communication
  def checksum_check(self, response):
    # check if response contains data
    if len(response):
      # if so then check if received response is valid
      if self.checksum_8(response[:-1]) != response[-1]:
        return 1
      # if so then check if previously sent command was received correctly
      return (response[0] & self.CKSUM_ERR)
    else:
      return 1


  def checksum_8(self, bytes_arg):
    return sum(bytes_arg)%256


