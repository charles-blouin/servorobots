import smbus
import struct
import collections

class Regs(object):
  CTRL1_XL    = 0x10
  CTRL2_G     = 0x11
  CTRL3_C     = 0x12
  OUTX_L_G    = 0x22
  OUTX_L_XL   = 0x28

Vector = ('Vector', 'x y z')

class LSM6(object):

  def __init__(self, slave_addr = 0b1101011):
    self.bus = smbus.SMBus(1)
    self.sa = slave_addr
    # self.g = Vector(0, 0, 0)
    # self.a = Vector(0, 0, 0)
      
  def enable(self):
    self.bus.write_byte_data(self.sa, Regs.CTRL1_XL, 0x50) # 208 Hz ODR, 2 g FS
    self.bus.write_byte_data(self.sa, Regs.CTRL2_G, 0x58) # 208 Hz ODR, 1000 dps FS
    self.bus.write_byte_data(self.sa, Regs.CTRL3_C, 0x04) # IF_INC = 1 (automatically increment register address)

  def read_gyro(self):
    byte_list = self.bus.read_i2c_block_data(self.sa, Regs.OUTX_L_G, 6)
    # The maximum is max int16. The factor is 1/32767*1000 dps/360*2*pi = 0.00053264802
    self.gx, self.gy, self.gz = struct.unpack('hhh', bytes(byte_list))
    self.gx *= 0.00053264802
    self.gy *= 0.00053264802
    self.gz *= 0.00053264802
    

  def read_accel(self):
    byte_list = self.bus.read_i2c_block_data(self.sa, Regs.OUTX_L_XL, 6)
    # The factor is 1/32767*2*9.81 = 0.00059877315
    self.ax, self.ay, self.az = struct.unpack('hhh', bytes(byte_list))
    self.ax *= 0.00059877315
    self.ay *= 0.00059877315
    self.az *= 0.00059877315
    

  def read(self):
    self.read_gyro()
    self.read_accel()