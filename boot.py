# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal
import sys
import machine
import pyb
import os
import time
import sdcard
from machine import I2C, UART, SPI, Pin

pyb.country('ES') # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU
#pyb.main('main.py') # main script to run after this one
#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device
#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse

#Acces facil als Pins
pin = Pin.board


#Montar la SD

sd = sdcard.SDCard(SPI(1), pin.PB0)
try:
    os.mount(sd, '/sd')
    #donar acces a l'import per la sd
    sys.path.append('/sd')
except:
    print("Hi ha hagut un problema muntant la SD, verificar")
### Per a veure el directori fer:
### os.listdir('/sd')


