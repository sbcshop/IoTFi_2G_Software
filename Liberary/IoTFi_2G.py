'''
#------------------------------------------------------------------------             
# This is a python install script written for qwiic python package.         
     
# Written by  SparkFun Electronics, May 2019
# This python library supports the SparkFun Electroncis qwiic 
# ecosystem, providing an plaform indepenant interface to the 
        
        
          
# I2C bus. 
        
        
          
#
        
        
          
# More information on qwiic is at https://www.sparkfun.com/qwiic
        
        
          
#
        
        
          
# Do you like this library? Help support SparkFun. Buy a board!
        
        
          
#
        
        
          
#==================================================================================
        
        
          
# Copyright (c) 2019 SparkFun Electronics
        
        
          
#
        
        
          
# Permission is hereby granted, free of charge, to any person obtaining a copy 
        
        
          
# of this software and associated documentation files (the "Software"), to deal 
        
        
          
# in the Software without restriction, including without limitation the rights 
        
        
          
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
        
        
          
# copies of the Software, and to permit persons to whom the Software is 
        
        
          
# furnished to do so, subject to the following conditions:
        
        
          
#
        
        
          
# The above copyright notice and this permission notice shall be included in all    
# copies or substantial portions of the Software.
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR       
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER    
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,     
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE      
# SOFTWARE.   
#==================================================================================
'''

import random
from machine import Pin, UART, SPI, I2C, PWM
import os
import utime
import time
import binascii
import framebuf

DEVICE_ADDRESS = 0x12 #Device address of accelerometer

pwr_enable = 29  # SIM868 Power key connected on GP22
uart_port = 0
uart_baud = 9600

# Initializing UART0

uart = UART(uart_port, uart_baud) #SIM868 UART 
print(os.uname())


############ QMA7981 Module #######
##################################


i2c = I2C(0,scl=Pin(21),sda=Pin(20)) #Initialisation of i2c for Accelerometer

'''
qma7981_full_scale_range_t
    RANGE_2G = 0b0001,
    RANGE_4G = 0b0010,
    RANGE_8G = 0b0100,
    RANGE_16G = 0b1000,
    RANGE_32G = 0b1111

qma7981_bandwidth
    MCLK_DIV_BY_7695 = 0b000,
    MCLK_DIV_BY_3855 = 0b001,
    MCLK_DIV_BY_1935 = 0b010,
    MCLK_DIV_BY_975 = 0b011,
    MCLK_DIV_BY_15375 = 0b101,
    MCLK_DIV_BY_30735 = 0b110,
    MCLK_DIV_BY_61455 = 0b111

qma7981_clock_freq_t
    CLK_500_KHZ = 0b0001,
    CLK_333_KHZ = 0b0000,
    CLK_200_KHZ = 0b0010,
    CLK_100_KHZ = 0b0011,
    CLK_50_KHZ = 0b0100,
    CLK_25_KHZ = 0b0101,
    CLK_12_KHZ_5 = 0b0110,
    CLK_5_KHZ = 0b0111
'''

CLK = 0b0100
bandwidth = 0b010 # bandwidth
scale_range = 0b1111

devices = i2c.scan()
if len(devices) == 0:
 print("No i2c device !")
else:
 print('i2c devices found:',len(devices))
for device in devices:
    print("At address: ",hex(device))
    if device == DEVICE_ADDRESS :
        print("device found")
    else :
        print("device not found")

class accelerometer(object):
    def __init__(self):
        self.i2c = i2c
    def reg_write(self,i2c, addr, reg, data):
        """
        Write bytes to the specified register.
        """
        # Construct message
        msg = bytearray()
        msg.append(data)
        # Write out message to register
        self.i2c.writeto_mem(addr, reg, msg)
        
    def reg_read(self,i2c, addr, reg, nbytes=1):
        """
        Read byte(s) from specified register. If nbytes > 1, read from consecutive
        registers.
        """
        # Check to make sure caller is asking for 1 or more bytes
        if nbytes < 1:
            return bytearray()
        # Request data from specified register(s) over I2C
        data = self.i2c.readfrom_mem(addr, reg, nbytes)
        return data

    def read_x(self):
        acc_l = self.reg_read(i2c,0x12, 0x01, 1)
        acc_h = self.reg_read(i2c,0x12, 0x02, 1)
        acc_combined = ((ord(acc_l) & 252) | ord(acc_h) <<8) #ord is to get int representation from a byte
        acc_combined = acc_combined/4
        #print(acc_combined)
        return acc_combined  if acc_combined < 8191 else acc_combined - 16383
        
    def read_y(self):
        acc_l = self.reg_read(i2c,0x12, 0x03, 1)
        acc_h = self.reg_read(i2c,0x12, 0x04, 1)
        acc_combined = ((ord(acc_l) & 252) | ord(acc_h) <<8) #ord is to get int representation from a byte
        acc_combined = acc_combined/4
        return acc_combined  if acc_combined < 8191 else acc_combined - 16383

    def read_z(self):
        acc_l = self.reg_read(i2c,0x12, 0x05, 1)
        acc_h = self.reg_read(i2c,0x12, 0x06, 1)
        acc_combined = ((ord(acc_l) & 252) | ord(acc_h) <<8) #ord is to get int representation from a byte
        acc_combined = acc_combined/4
        return acc_combined  if acc_combined < 8191 else acc_combined - 16383

    def set_mode(self):
        reg = self.reg_read(i2c,0x12, 0x11)
        self.reg_write(i2c,0x12, 0x11, 0xF4) 

    def soft_reset(self):
        self.reg_write(i2c,0x12, 0x36, 0xB6)    #softreset device
        self.reg_write(i2c,0x12, 0x36, 0x00)    #softreset device

    def set_clock_freq(self):
        reg = self.reg_read(i2c,0x12, 0x11)
        print(reg)
        reg = ord(reg) & (0b11110000)    # clear bits 0-3
        reg = reg | (CLK & 0b1111); # set freq on bits 0-3
        self.reg_write(i2c,0x12, 0x11, reg) 

    def set_bandwidth(self):
        data = 0b11100000
        data = data | (bandwidth &  0b111)
        self.reg_write(i2c,0x12, 0x10, data) 

    def set_full_scale_range(self):
        data = 0b11110000
        data = data & (scale_range & 0b1111)
        self.reg_write(i2c,0x12, 0x0F, data) 
        
    def initialize(self):
        print("########")
        time.sleep(0.05)
        self.soft_reset()
        time.sleep(0.05)
        self.set_mode()
        time.sleep(0.05)
        self.set_clock_freq()
        time.sleep(0.05)
        self.set_bandwidth()
        time.sleep(2)


############ SIM868 module #######
##################################
        
class sim868:
    def __init__(self):
        self.uart = UART(0, 9600)
        
    def wait_resp_info(self, timeout=3000):
        prvmills = utime.ticks_ms()
        info = b""
        while (utime.ticks_ms()-prvmills) < timeout:
            if self.uart.any():
                info = b"".join([info, self.uart.read(1)])
        print(info.decode())
        return info
    
    def Send_command(self, cmd, back, timeout=2000):  # Send AT command
        rec_buff = b''
        self.uart.write((cmd+'\r\n').encode())
        prvmills = utime.ticks_ms()
        while (utime.ticks_ms()-prvmills) < timeout:
            if self.uart.any():
                rec_buff = b"".join([rec_buff, self.uart.read(1)])
        if rec_buff != '':
            if back not in rec_buff.decode():
                print(cmd + ' back:\t' + rec_buff.decode())
                return 0
            else:
                print(rec_buff.decode())
                return 1
        else:
            print(cmd + ' no responce')
            
    def Send_command_wait_resp(self, cmd, back, timeout=2000): # Send AT command and return response information
        rec_buff = b''
        self.uart.write((cmd + '\r\n').encode())
        prvmills = utime.ticks_ms()
        while (utime.ticks_ms() - prvmills) < timeout:
            if self.uart.any():
                rec_buff = b"".join([rec_buff, self.uart.read(1)])
        if rec_buff != '':
            if back not in rec_buff.decode():
                print(cmd + ' back:\t' + rec_buff.decode())
            else:
                print(rec_buff.decode())
        else:
            print(cmd + ' no responce')
        # print("Response information is: ", rec_buff)
        return rec_buff
    
    def Check_and_start(self): # Initialize SIM Module 
        while True:
            self.uart.write(bytearray(b'ATE1\r\n'))
            utime.sleep(2)
            self.uart.write(bytearray(b'AT\r\n'))
            rec_temp = self.wait_resp_info()
            if 'OK' in rec_temp.decode():
                print('Pico 2G is ready\r\n' + rec_temp.decode())
                #LCD.fill(LCD.black) 
                #LCD.text("Pico 2G is ready",40,40,LCD.white) 
                #LCD.lcd_show()
                break
            else:
                power = Pin(pwr_enable, Pin.OUT)
                power.value(1)
                utime.sleep(2)
                power.value(0)
                print('Pico 2G is starting up, please wait...\r\n')
                #LCD.fill(LCD.black) 
                #LCD.text("Pico 2G is starting up",40,40,LCD.white)
                #LCD.text("Please wait...",40,60,LCD.white) 
                #LCD.lcd_show()
                utime.sleep(4)
    def Network_checking(self):# Network connectivity check
        for i in range(1, 3):
            if self.Send_command("AT+CGREG?", "0,1") == 1:
                print('SIM868 is online\r\n')
                break
            else:
                print('SIM868 is offline, please wait...\r\n')
                utime.sleep(2)
                continue
    def gps(self):
        self.Check_and_start()
        count = 0
        print('Start GPS...')
        self.Send_command('AT+CGNSPWR=1', 'OK')
        #LCD.fill(LCD.black) 
        #LCD.lcd_show()

        #LCD.text("GPS POWER ON",40,40,LCD.white) 
        #LCD.lcd_show()
        utime.sleep(2)
        for i in range(1, 10):
            self.uart.write(bytearray(b'AT+CGNSINF\r\n'))
            rec_buff = self.wait_resp_info()
            if ',,,,' in rec_buff.decode():
                print('GPS is not ready')
                #LCD.fill(LCD.black) 
                #LCD.lcd_show()
                #LCD.text("GPS is not ready",40,60,LCD.white) 
                #LCD.lcd_show()
                utime.sleep(5)
                #print(rec_buff.decode())
                if i >= 9:
                    print('GPS positioning failed, please check the GPS antenna!\r\n')
                    self.Send_command('AT+CGNSPWR=0', 'OK')
                    #LCD.fill(LCD.black) 
                    #LCD.lcd_show()

                    #LCD.text("GPS positioning failed",40,40,LCD.white)
                    #LCD.text("GPS POWER OFF",40,60,LCD.white) 
                    #LCD.lcd_show()
                    utime.sleep(4)
                else:
                    utime.sleep(2)
                    continue
            else:
                if count <= 3:
                    count += 1
                    print('GPS info:')
                    print(rec_buff.decode())
                else:
                    self.Send_command('AT+CGNSPWR=0', 'OK')
                    #LCD.fill(LCD.black) 
                    #LCD.lcd_show()
                    #LCD.text("GPS POWER OFF",40,60,LCD.white) 
                    #LCD.lcd_show()
                    utime.sleep(4)
                    break
                
    def call(self,mobile_number,time):
        self.Check_and_start() # Initialize SIM Module 
        self.Network_checking() # Network connectivity check
        
        #Send_command('AT+CHFA=2', 'OK')
        self.Send_command("ATD"+mobile_number+";", 'OK')
        utime.sleep(time)
        self.Send_command('AT+CHUP;', 'OK')


    def message(self, phone_num, sms_text):
        def Hex_str_to_str(hex_str):
            hex_data = hex_str.encode('utf-8')
            str_bin = binascii.unhexlify(hex_data)
            return str_bin.decode('utf-8')
        
        
        def Str_to_hex_str(string):
            str_bin = string.encode('utf-8')
            return binascii.hexlify(str_bin).decode('utf-8')
        
        
           # Send SMS function
        self.Check_and_start() 
        self.Network_checking()
        self.Send_command('AT+CMGF=1', 'OK')
        if self.Send_command('AT+CMGS=\"'+phone_num+'\"', '>'):
                self.uart.write(bytearray(sms_text))
                utime.sleep(0.5)
                self.uart.write(bytearray(Hex_str_to_str("1A")))
               
               
    def bluetooth(): 
        Check_and_start() 
        Send_command('AT+BTPOWER=1', 'OK', 3000)
        LCD.fill(LCD.black) 
        LCD.lcd_show()
        LCD.text("BT power on",40,40,LCD.white) 
        LCD.lcd_show()
        
        Send_command('AT+BTHOST?', 'OK', 3000)
        Send_command('AT+BTSTATUS?', 'OK', 3000)
        Send_command('AT+BTSCAN=1,10', 'OK', 8000)
        LCD.fill(LCD.black) 
        LCD.lcd_show()
        LCD.text("Scan BT Devices",40,40,LCD.white) 
        LCD.lcd_show()
        
        utime.sleep(5)
        Send_command('AT+BTPOWER=0', 'OK')
        LCD.fill(LCD.black) 
        LCD.lcd_show()
        LCD.text("BT power off",40,40,LCD.white) 
        LCD.lcd_show()


########## ESP32-C3 Module ##############
########################################


class wifi():
    lst = []
    def __init__(self,wifi_ssid,wifi_pass,wifi_port):
        self.uart = UART(1, 115200) 
        self.wifi_ssid = wifi_ssid
        self.wifi_pass = wifi_pass
        self.wifi_port = wifi_port

    def sendCMD(self,cmd,ack,timeout=2000):
        self.uart.write(cmd+'\r\n')
        t = utime.ticks_ms()
        while (utime.ticks_ms() - t) < timeout:
            s=self.uart.read()
            if(s != None):
                s=s.decode()
                print(s)
                if cmd =="AT+CIFSR":
                    self.lst.append(s)            
                if(s.find(ack) >= 0):
                    return True
        return False

    def sendData(ID,data):
        self.sendCMD('AT+CIPSEND='+str(ID)+','+str(len(data)),'>')
        self.uart.write(data)

    def ReceiveData(self):
        data=self.uart.read()
        if(data != None):
            data=data.decode()
            print(data)
            if(data.find('+IPD') >= 0):
                n1=data.find('+IPD,')
                n2=data.find(',',n1+5)
                ID=int(data[n1+5:n2])
                n3=data.find(':')
                data=data[n3+1:]
                return ID,data
        return None,None
        
    def Wifi_start(self):
        self.uart.write('+++')
        time.sleep(1)
        if(self.uart.any()>0):self.uart.read()
        self.sendCMD("AT","OK")
        self.sendCMD("AT+CWMODE=3","OK")
        self.sendCMD("AT+CWJAP=\""+self.wifi_ssid+"\",\""+self.wifi_pass+"\"","OK",20000)
        self.sendCMD("AT+CIPMUX=1","OK")
        self.sendCMD("AT+CIPSERVER=1,"+self.wifi_port,"OK")
        self.sendCMD("AT+CIFSR","OK")

########## SD Card module ##############
########################################
        
class SDCard:
    _CMD_TIMEOUT = const(100)
    _R1_IDLE_STATE = const(1 << 0)
    _R1_ILLEGAL_COMMAND = const(1 << 2)
    _TOKEN_CMD25 = const(0xFC)
    _TOKEN_STOP_TRAN = const(0xFD)
    _TOKEN_DATA = const(0xFE)
    def __init__(self):
        spi=SPI(0,sck=Pin(18),mosi=Pin(19),miso=Pin(16))
        cs = Pin(17)
        self.spi = spi
        self.cs = cs

        self.cmdbuf = bytearray(6)
        self.dummybuf = bytearray(512)
        self.tokenbuf = bytearray(1)
        for i in range(512):
            self.dummybuf[i] = 0xFF
        self.dummybuf_memoryview = memoryview(self.dummybuf)
        # initialise the card
        self.init_card()

    def init_spi(self, baudrate):
        try:
            master = self.spi.MASTER
        except AttributeError:
            # on ESP8266
            self.spi.init(baudrate=baudrate, phase=0, polarity=0)
        else:
            # on pyboard
            self.spi.init(master, baudrate=baudrate, phase=0, polarity=0)

    def init_card(self):
        # init CS pin
        self.cs.init(self.cs.OUT, value=1)

        # init SPI bus; use low data rate for initialisation
        self.init_spi(100000)

        # clock card at least 100 cycles with cs high
        for i in range(16):
            self.spi.write(b"\xff")

        # CMD0: init card; should return _R1_IDLE_STATE (allow 5 attempts)
        for _ in range(5):
            if self.cmd(0, 0, 0x95) == _R1_IDLE_STATE:
                break
        else:
            raise OSError("no SD card")

        # CMD8: determine card version
        r = self.cmd(8, 0x01AA, 0x87, 4)
        if r == _R1_IDLE_STATE:
            self.init_card_v2()
        elif r == (_R1_IDLE_STATE | _R1_ILLEGAL_COMMAND):
            self.init_card_v1()
        else:
            raise OSError("couldn't determine SD card version")

        # get the number of sectors
        # CMD9: response R2 (R1 byte + 16-byte block read)
        if self.cmd(9, 0, 0, 0, False) != 0:
            raise OSError("no response from SD card")
        csd = bytearray(16)
        self.readinto(csd)
        if csd[0] & 0xC0 == 0x40:  # CSD version 2.0
            self.sectors = ((csd[8] << 8 | csd[9]) + 1) * 1024
        elif csd[0] & 0xC0 == 0x00:  # CSD version 1.0 (old, <=2GB)
            c_size = csd[6] & 0b11 | csd[7] << 2 | (csd[8] & 0b11000000) << 4
            c_size_mult = ((csd[9] & 0b11) << 1) | csd[10] >> 7
            self.sectors = (c_size + 1) * (2 ** (c_size_mult + 2))
        else:
            raise OSError("SD card CSD format not supported")
        # print('sectors', self.sectors)

        # CMD16: set block length to 512 bytes
        if self.cmd(16, 512, 0) != 0:
            raise OSError("can't set 512 block size")

        # set to high data rate now that it's initialised
        self.init_spi(1320000)

    def init_card_v1(self):
        for i in range(_CMD_TIMEOUT):
            self.cmd(55, 0, 0)
            if self.cmd(41, 0, 0) == 0:
                self.cdv = 512
                # print("[SDCard] v1 card")
                return
        raise OSError("timeout waiting for v1 card")

    def init_card_v2(self):
        for i in range(_CMD_TIMEOUT):
            time.sleep_ms(50)
            self.cmd(58, 0, 0, 4)
            self.cmd(55, 0, 0)
            if self.cmd(41, 0x40000000, 0) == 0:
                self.cmd(58, 0, 0, 4)
                self.cdv = 1
                # print("[SDCard] v2 card")
                return
        raise OSError("timeout waiting for v2 card")

    def cmd(self, cmd, arg, crc, final=0, release=True, skip1=False):
        self.cs(0)

        # create and send the command
        buf = self.cmdbuf
        buf[0] = 0x40 | cmd
        buf[1] = arg >> 24
        buf[2] = arg >> 16
        buf[3] = arg >> 8
        buf[4] = arg
        buf[5] = crc
        self.spi.write(buf)

        if skip1:
            self.spi.readinto(self.tokenbuf, 0xFF)

        # wait for the response (response[7] == 0)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xFF)
            response = self.tokenbuf[0]
            if not (response & 0x80):
                # this could be a big-endian integer that we are getting here
                for j in range(final):
                    self.spi.write(b"\xff")
                if release:
                    self.cs(1)
                    self.spi.write(b"\xff")
                return response

        # timeout
        self.cs(1)
        self.spi.write(b"\xff")
        return -1

    def readinto(self, buf):
        self.cs(0)

        # read until start byte (0xff)
        for i in range(_CMD_TIMEOUT):
            self.spi.readinto(self.tokenbuf, 0xFF)
            if self.tokenbuf[0] == _TOKEN_DATA:
                break
            time.sleep_ms(1)
        else:
            self.cs(1)
            raise OSError("timeout waiting for response")

        # read data
        mv = self.dummybuf_memoryview
        if len(buf) != len(mv):
            mv = mv[: len(buf)]
        self.spi.write_readinto(mv, buf)

        # read checksum
        self.spi.write(b"\xff")
        self.spi.write(b"\xff")

        self.cs(1)
        self.spi.write(b"\xff")

    def write(self, token, buf):
        self.cs(0)

        # send: start of block, data, checksum
        self.spi.read(1, token)
        self.spi.write(buf)
        self.spi.write(b"\xff")
        self.spi.write(b"\xff")

        # check the response
        if (self.spi.read(1, 0xFF)[0] & 0x1F) != 0x05:
            self.cs(1)
            self.spi.write(b"\xff")
            return

        # wait for write to finish
        while self.spi.read(1, 0xFF)[0] == 0:
            pass

        self.cs(1)
        self.spi.write(b"\xff")

    def write_token(self, token):
        self.cs(0)
        self.spi.read(1, token)
        self.spi.write(b"\xff")
        # wait for write to finish
        while self.spi.read(1, 0xFF)[0] == 0x00:
            pass

        self.cs(1)
        self.spi.write(b"\xff")

    def readblocks(self, block_num, buf):
        nblocks = len(buf) // 512
        assert nblocks and not len(buf) % 512, "Buffer length is invalid"
        if nblocks == 1:
            # CMD17: set read address for single block
            if self.cmd(17, block_num * self.cdv, 0, release=False) != 0:
                # release the card
                self.cs(1)
                raise OSError(5)  # EIO
            # receive the data and release card
            self.readinto(buf)
        else:
            # CMD18: set read address for multiple blocks
            if self.cmd(18, block_num * self.cdv, 0, release=False) != 0:
                # release the card
                self.cs(1)
                raise OSError(5)  # EIO
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                # receive the data and release card
                self.readinto(mv[offset : offset + 512])
                offset += 512
                nblocks -= 1
            if self.cmd(12, 0, 0xFF, skip1=True):
                raise OSError(5)  # EIO

    def writeblocks(self, block_num, buf):
        nblocks, err = divmod(len(buf), 512)
        assert nblocks and not err, "Buffer length is invalid"
        if nblocks == 1:
            # CMD24: set write address for single block
            if self.cmd(24, block_num * self.cdv, 0) != 0:
                raise OSError(5)  # EIO

            # send the data
            self.write(_TOKEN_DATA, buf)
        else:
            # CMD25: set write address for first block
            if self.cmd(25, block_num * self.cdv, 0) != 0:
                raise OSError(5)  # EIO
            # send the data
            offset = 0
            mv = memoryview(buf)
            while nblocks:
                self.write(_TOKEN_CMD25, mv[offset : offset + 512])
                offset += 512
                nblocks -= 1
            self.write_token(_TOKEN_STOP_TRAN)

    def ioctl(self, op, arg):
        if op == 4:  # get number of blocks
            return self.sectors

###################### LCD1_14 Driver ###############################
#####################################################################
        
class Lcd1_14(framebuf.FrameBuffer):
    def __init__(self):
        self.width = 240 #
        self.height = 135
        
        self.cs = Pin(9,Pin.OUT)
        self.rst = Pin(12,Pin.OUT)
        
        self.cs(1)
        self.spi = SPI(1)
        self.spi = SPI(1,1000_000)
        self.spi = SPI(1,10000_000,polarity=0, phase=0,sck=Pin(10),mosi=Pin(11),miso=None)
        self.dc = Pin(8,Pin.OUT)
        self.dc(1)
        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)  #Red Green Blue (16-bit, 5+6+5) color format
        self.lcd_init_display()
        
        self.red   =   0x07E0
        self.green =   0x001f
        self.blue  =   0xf800
        self.white =   0xffff
        self.black =   0x0000
        
    def lcd_write_cmd(self, cmd):
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def lcd_write_data(self, buf):
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(bytearray([buf]))
        self.cs(1)

    def lcd_init_display(self):
        """Initialize dispaly"""  
        self.rst(1)
        self.rst(0)
        self.rst(1)
        
        self.lcd_write_cmd(0x36)
        self.lcd_write_data(0x70)

        self.lcd_write_cmd(0x3A) 
        self.lcd_write_data(0x05)

        self.lcd_write_cmd(0xB2)
        self.lcd_write_data(0x0C)
        self.lcd_write_data(0x0C)
        self.lcd_write_data(0x00)
        self.lcd_write_data(0x33)
        self.lcd_write_data(0x33)

        self.lcd_write_cmd(0xB7)
        self.lcd_write_data(0x35) 

        self.lcd_write_cmd(0xBB)
        self.lcd_write_data(0x19)

        self.lcd_write_cmd(0xC0)
        self.lcd_write_data(0x2C)

        self.lcd_write_cmd(0xC2)
        self.lcd_write_data(0x01)

        self.lcd_write_cmd(0xC3)
        self.lcd_write_data(0x12)   

        self.lcd_write_cmd(0xC4)
        self.lcd_write_data(0x20)

        self.lcd_write_cmd(0xC6)
        self.lcd_write_data(0x0F) 

        self.lcd_write_cmd(0xD0)
        self.lcd_write_data(0xA4)
        self.lcd_write_data(0xA1)

        self.lcd_write_cmd(0xE0)
        self.lcd_write_data(0xD0)
        self.lcd_write_data(0x04)
        self.lcd_write_data(0x0D)
        self.lcd_write_data(0x11)
        self.lcd_write_data(0x13)
        self.lcd_write_data(0x2B)
        self.lcd_write_data(0x3F)
        self.lcd_write_data(0x54)
        self.lcd_write_data(0x4C)
        self.lcd_write_data(0x18)
        self.lcd_write_data(0x0D)
        self.lcd_write_data(0x0B)
        self.lcd_write_data(0x1F)
        self.lcd_write_data(0x23)

        self.lcd_write_cmd(0xE1)
        self.lcd_write_data(0xD0)
        self.lcd_write_data(0x04)
        self.lcd_write_data(0x0C)
        self.lcd_write_data(0x11)
        self.lcd_write_data(0x13)
        self.lcd_write_data(0x2C)
        self.lcd_write_data(0x3F)
        self.lcd_write_data(0x44)
        self.lcd_write_data(0x51)
        self.lcd_write_data(0x2F)
        self.lcd_write_data(0x1F)
        self.lcd_write_data(0x1F)
        self.lcd_write_data(0x20)
        self.lcd_write_data(0x23)
        
        self.lcd_write_cmd(0x21)

        self.lcd_write_cmd(0x11)

        self.lcd_write_cmd(0x29)

    def lcd_show(self):
        self.lcd_write_cmd(0x2A)
        self.lcd_write_data(0x00)
        self.lcd_write_data(0x28)
        self.lcd_write_data(0x01)
        self.lcd_write_data(0x17)
        
        self.lcd_write_cmd(0x2B)
        self.lcd_write_data(0x00)
        self.lcd_write_data(0x35)
        self.lcd_write_data(0x00)
        self.lcd_write_data(0xBB)
        
        self.lcd_write_cmd(0x2C)
        
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)

