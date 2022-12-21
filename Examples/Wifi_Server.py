'''
This is the example code for working with WiFi functionality of IoTFi board
Developed by sbcomponents
'''

from IoTFi_2G import wifi, Lcd1_14
import time

LCD = Lcd1_14()

#uart = UART(1, 9600) 

def infoDevice():
        LCD.fill(LCD.black) 
        LCD.hline(10,10,220,LCD.white)
        LCD.hline(10,125,220,LCD.white)
        LCD.vline(10,10,115,LCD.white)
        LCD.vline(230,10,115,LCD.white)       
        
        LCD.text("SB-COMPONENTS",70,40,LCD.white)
        LCD.text("IoTFi 4G",70,60,LCD.white)
        LCD.lcd_show()
        time.sleep(2)
        LCD.fill(LCD.black)
        LCD.text("SERVER.....",70,40,LCD.white)
        LCD.lcd_show()
        x = 0
        for y in range(0,1):
                x += 4
                LCD.text(".",125+x,40,LCD.white)
                LCD.lcd_show()
                time.sleep(1)

infoDevice()

wifi_ssid = " "  #Enter your ssid name
wifi_pass = " "  #Enter your password
wifi_port = "8080"  #Server port

server = wifi(wifi_ssid, wifi_pass, wifi_port)
server.Wifi_start()
LCD.fill(LCD.black)
