import serial
import time
import sys

class nmccom():

  def __init__(self):
    self.baud_code = {}
    self.baud_code[9600] = 127
    self.baud_code[19200] = 64
    self.baud_code[57600] = 21
    self.baud_code[115200] = 10
    self.baud_code[230400] = 5

    self.move_done = 0x01
    self.cksum_err = 0x02
    self.pwr_on = 0x04
    self.pos_err = 0x08
    self.limit1 = 0x10
    self.limit2 = 0x20
    self.home_in_prog = 0x40
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
      self.port.write(bytes(20))
      self.port.flushInput()
      time.sleep(1.0)

      self.simple_reset()
      addr = 0
      while self.set_addr(0x00, addr+1, 0xFF):
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
      status_dict = self.get_full_status(addr)
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


  def set_addr(self, curr_addr, new_addr, group):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x %02x' % (curr_addr, 0x21, new_addr, group)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('set_addr response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error setting module address\n')
    return resp

  
  def simple_reset(self):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x' % (0xFF, 0x0F)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('simple_reset response ', resp)


  def hard_reset(self, module):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x' % (module, 0x1F, 0x00)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('hard_reset response ', resp)


  def set_pos(self, module, pos):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x %08x' % (module, 0x50, 0x02, pos)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('set_pos response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error setting position\n')

  
  def reset_pos(self, module):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x' % (module, 0x00)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('reset_pos response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error resetting position\n')


  def define_status(self, module, status_bits):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x' % (module, 0x12, status_bits)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('define_status response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error defining status\n')


  def get_pos(self, module):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x' % (module, 0x13, 0x01)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status + 4)
    if self.checksum_error(resp):
      sys.stderr.write('Error getting status\n')
    return resp


  def get_full_status(self, module):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x' % (module, 0x13, 0xFF)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status + 17)
    if self.checksum_error(resp):
      sys.stderr.write('Error getting status\n')

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
    return status_dict


  def set_baud(self, baudrate):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x' % (0xFF, self.baud_code[baudrate])))
    self.port.write(cmd)
#    self.port.close()
    self.port.baudrate = baudrate
#    self.port.open()
    resp = self.port.read(self.len_status)
    print('set_baud response ', resp)


  def make_cmd(self, bytes_arg):
    cmd = bytes([0xAA]) + bytes_arg + bytes([self.checksum8(bytes_arg)])
    return cmd


  def checksum_error(self, response):
    if len(response):
      if self.checksum8(response[:-1]) != response[-1]:
        return 1
      return (response[0] & self.cksum_err) > 0
    else:
      return 1


  def checksum8(self, bytes_arg):
    return sum(bytes_arg)%256

