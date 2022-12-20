from machine import Pin, SPI
import os
from IoTFi_2G import accelerometer, SDCard, Lcd1_14
import time 

axi = accelerometer()  #Accelerometer driver
axi.initialize()       #Accelerometer initiallisation
sd=SDCard()            #sdcard driver
LCD = Lcd1_14()        #driver of lcd display

x_axis = []        # the x, y, and z axis data is collected in 3 lists  
y_axis = []
z_axis = []

def info():
    LCD.text("IoTFi 2G", 10,10,LCD.blue)# print on tft screen
    LCD.fill_rect(0, 40, 240,5, LCD.red)#display red line on tft screen
    LCD.lcd_show()
    LCD.text("ACCELEROMETER", 10,70 ,LCD.green)
    LCD.fill_rect(0, 105, 240,5, LCD.red)
    LCD.lcd_show()
info()
time.sleep(2)

LCD.fill(0)

#while True:
x = str(axi.read_x()/2)
y = str(axi.read_y()/2)
z = str(axi.read_z()/2)

time.sleep(0.05)

x_axis.append(x)     # writing new value in each list
y_axis.append(y)
z_axis.append(z)


def file_exists(fn):
  try:
    with open(fn):
      pass
    return True
  except OSError:
     return False

sd=SDCard()
vfs = os.VfsFat(sd)
os.mount(vfs, "/fc")
print("Filesystem check")
print(os.listdir("/fc")) # check the files in sd card
fn = "/fc/Sensor_Read.txt"
print("Single block read/write")

#################################################

with open(fn, "a") as f:  # append data to file
    n = f.write("{} \n".format(x))
    n = f.write("{} \n".format(y))
    n = f.write("{} \n".format(z))
    f.close()
    print(n, "bytes written")
    LCD.fill(0)
    LCD.text(str(n)+" bytes written", 10,50 ,LCD.green)
    LCD.lcd_show()
#################################################

#################################################
with open(fn, "r") as f:  # read data from file
    result = f.read()
    f.close()
    print(result)
    print(len(result), "bytes read")
    #LCD.fill(0)
    LCD.text(str(len(result))+" bytes read", 10,70 ,LCD.green)
    LCD.lcd_show()
os.umount("/fc")
#################################################

