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


  def init_nmc(self, port = '/dev/ttyUSB0', baudrate=19200, timeout = 0.01):
    
    self.port = serial.Serial(port, baudrate=baudrate, bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, timeout = timeout)

    # Clear nmc bus
    self.port.write(bytes(20))
    self.port.flushInput()
    time.sleep(0.01)

    self.len_status = 1 + 1

    self.simple_reset()
    addr = 0
    while self.set_addr(0x00, addr+1, 0xFF):
      addr += 1

    num_nmc = addr
    print('Found %d NMCs' % (num_nmc)) 
    for addr in range(1,num_nmc+1):
      print('Status of NMC %d: ' % (addr), self.get_full_status(addr))


  def set_addr(self, module, addr, group):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x %02x' % (module, 0x21, addr, group)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error setting module address\n')
    return resp

  
  def simple_reset(self):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x' % (0xFF, 0x0F)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)


  def set_pos(self, module, pos):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x %08x' % (module, 0x50, 0x02, pos)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error setting position\n')

  
  def reset_pos(self, module):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x' % (module, 0x00)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('response ', resp)
    if self.checksum_error(resp):
      sys.stderr.write('Error resetting position\n')


  def define_status(self, module, status_bits):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x %02x' % (module, 0x12, status_bits)))
    self.port.write(cmd)
    resp = self.port.read(self.len_status)
    print('response ', resp)
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
    return resp


  def set_baud(self, baudrate):
    cmd = self.make_cmd(bytearray.fromhex('%02x %02x' % (0xFF, self.baud_code[baudrate])))
    self.port.write(cmd)
    self.port.baudrate = baudrate
    resp = self.port.read(self.len_status)


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

