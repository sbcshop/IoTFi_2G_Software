from machine import Pin
import os
from IoTFi_2G import sim868, Lcd1_14
import time

LCD = Lcd1_14()#driver of lcd display

def infoDevice():
        LCD.fill(LCD.black) 
        LCD.hline(10,10,220,LCD.white)
        LCD.hline(10,125,220,LCD.white)
        LCD.vline(10,10,115,LCD.white)
        LCD.vline(230,10,115,LCD.white)       
        
        LCD.text("SB-COMPONENTS",70,40,LCD.white)
        LCD.text("IoTFi 2G",70,60,LCD.white)
        LCD.lcd_show()
        time.sleep(2)
        LCD.fill(LCD.black)
        LCD.text("WAITING.....",70,40,LCD.white)
        LCD.lcd_show()
        x = 0
        for y in range(0,1):
                x += 4
                LCD.text(".",125+x,40,LCD.white)
                LCD.lcd_show()
                time.sleep(2)
                LCD.fill(LCD.black)

infoDevice()

mobile_number = "Enter your mobile number"
time = 100
sms_text = "Hello from gsm test"

b1 = Pin(22, Pin.IN, Pin.PULL_UP)
b2 = Pin(23,Pin.IN, Pin.PULL_UP)

val1 = b1.value()
val2 = b2.value()

print("Press GP22 button to send message")
print("Press GP23 button to call")

    
while True:
    if b1.value() == 0:
        print("Sending msg....")
        LCD.text("Sending...",70,40,LCD.white)
        LCD.lcd_show()
        Message = sim868().message(mobile_number,sms_text)
        break
    
    if b2.value() == 0:
        print("Calling....")
        LCD.text("Calling...",70,40,LCD.white)
        LCD.lcd_show()
        Call = sim868().call(mobile_number,time)
        break
    if b2.value() == 0 and b1.value() == 0:
        print("Getting Location")
        LCD.text("Locating...",70,40,LCD.white)
        LCD.lcd_show()
        Gps = sim868().gps()
        break

