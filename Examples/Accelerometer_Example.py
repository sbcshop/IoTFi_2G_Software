from machine import Pin, SPI
from IoTFi_2G import accelerometer, Lcd1_14
import time 

axi = accelerometer()
axi.initialize()
LCD = Lcd1_14()
def info():
    
    LCD.fill(LCD.black) 
    LCD.hline(10,10,220,LCD.white)
    LCD.hline(10,125,220,LCD.white)
    LCD.vline(10,10,115,LCD.white)
    LCD.vline(230,10,115,LCD.white)
    
    LCD.text("IoTFi 2G", 90,40,LCD.blue)# print on tft screen
    LCD.lcd_show()
    LCD.hline(10,10,220,LCD.red)
    LCD.lcd_show()
    LCD.text("ACCELEROMETER", 70,60 ,LCD.green)
    LCD.hline(10,125,220,LCD.red)

    LCD.lcd_show()
info()
time.sleep(2)

LCD.fill(0)

while True:
    x = int(axi.read_x()/2)
    y = int(axi.read_y()/2)
    z = int(axi.read_z()/2)
    print(x, y, z)
    time.sleep(0.05)
    
    if y <= 90:
        LCD.fill(0)
        #tft.text(font,"Tilting Left", 30,60,st7789.RED)
        LCD.fill_rect(10, 50, 40, 40, LCD.red)
        LCD.lcd_show()
        print(x)
        print("Tilting Left")
        time.sleep(0.05)
        
    elif y >=175:
        LCD.fill(0)
        #tft.text(font,"Tilting Right", 30,60,st7789.RED)
        LCD.fill_rect(200, 50, 40, 40, LCD.red)
        LCD.lcd_show()
        print("Tilting Right")
        time.sleep(0.05)
    elif x >= 70:
        LCD.fill(0)
        #tft.text(font,"Tilting Frunt", 10,90,st7789.RED)
        LCD.fill_rect(100, 0, 40, 40, LCD.red)
        LCD.lcd_show()
        print(y)
        print("Tilting Forward")
        time.sleep(0.05)
        
    elif x <=-40:
        print("Tilting Backward")
        #tft.text(font,"Tilting Back", 10,90,st7789.RED)
        LCD.fill(0)
        LCD.fill_rect(100, 100, 40, 40, LCD.red)
        LCD.lcd_show()
        time.sleep(0.05)
    else:
         print("")
         LCD.fill(0)
         print("Please tilt")
         LCD.text("Tilt Please", 10,60,LCD.red)
         LCD.lcd_show()
        #tft.fill(0)
        #tft.fill_rect(100, 200, 40, 40, st7789.RED)
         time.sleep(0.05)
        
