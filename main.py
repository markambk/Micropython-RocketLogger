# main.py -- put your code here!
import machine
import time
import os
import pyb
import math
import bmp280
import mpu9250
from mpu6500 import MPU6500
from mpu9250 import MPU9250
from ak8963 import AK8963
from machine import I2C, UART, SPI, Pin
from math import log
from fusion import Fusion
from pyb import Switch
from orientate import orientate
from micropyGPS import MicropyGPS

#from micropyGPS import MicropyGPS
global offsetMag, scaleMag, sw, my_gps
sw = Switch()
SF_DEG_S = 1

i2c_gy91=I2C(sda= pin.PB7, scl=pin.PB6)
uart_gps = UART(2, baudrate=9600, bits=8, parity=None, stop=1)
my_gps = MicropyGPS()

try:
    from cal import *
    print("Dades de calibracio trobades, en cas de malfuncionament recalibrar amb calMPU()")
    MPUCalibrat = True
except:
    print("No es disposa de dades de calibracio, executa calMPU()")
    MPUCalibrat = False
    

start = pyb.millis()
def borrarlog():
    os.remove('/sd/log.csv')
    
def mesurar(vegades,step):
    log = open('/sd/log.csv', 'a')
    log.write('temps (s), pressio (Pa), altitud (m), acX, acY, acZ, gX, gY, gZ, mX, mY, mZ\n')
    log.close()
    for i in range(vegades):
        senBMP()
        senMPU()
        datalog()
        time.sleep_ms(step)
def debug(step):
    print('temps (s)\n')
    while True:
        senBMP()
        senMPU()
        datalog()
        t = pyb.elapsed_millis(start)/1000
        #print('{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(t,p,altitud,acX,acY,acZ,gX,gY,gZ,mX,mY,mZ))
        
        print('{},  {},  {}\n'.format(t,altitud,mag))
        time.sleep_ms(step)

#Data logger
def datalog():
    log = open('/sd/log.csv', 'a')
    t = pyb.elapsed_millis(start)/1000
    log.write('{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(t,p,altitud,acX,acY,acZ,gX,gY,gZ,mX,mY,mZ))
    log.close()
    
#Obtencio de dades de pressio
def senBMP():
    bmp = bmp280.BMP280(i2c_gy91)
    global p, altitud
    p = bmp.pressure
    altitud = (log(101325/p))/0.00012
    
#Obtencio de dades d'accelerometre, gyro i magnetometre.
def senMPU():
    global accel,acX,acY,acZ, gyro,gX,gY,gZ, mag,mX,mY,mZ
    
    bypass = MPU9250(i2c_gy91)
    if MPUCalibrat:
        ak8963 = AK8963(i2c_gy91, offset=offsetMag, scale=scaleMag)
        #Pendent d'actualització de la llibreria a la versio 4.0
        #mpu6500 = MPU6500(i2c_gy91, offset=offsetGyro, gyro_sf=SF_DEG_S) 
        mpu6500 = MPU6500(i2c_gy91, gyro_sf=SF_DEG_S)
    else:
        ak8963 = AK8963(i2c_gy91)
        mpu6500 = MPU6500(i2c_gy91, gyro_sf=SF_DEG_S)
    mpu = mpu9250.MPU9250(i2c_gy91, ak8963=ak8963, mpu6500=mpu6500)
    
    #Reorientacio dels eixos del accelerometre i el giroscopi per a que tinguin la mateixa disposicio que el magnetometre
    #Veure eixos al datasheet MPU9250
    accel, gyro = orientate((1,0,2), (False,False,True), mpu.acceleration, mpu.gyro)
    acX,acY,acZ = accel
    gX,gY,gZ = gyro
    
    mag = mpu.magnetic
    mX,mY,mZ = mag
    
    

#Calibracio MPU9250
def calMPU():
    #Calibració magnetometre
    bypass = MPU9250(i2c_gy91)
    ak8963 = AK8963(i2c_gy91)
    offsetMag, scaleMag = ak8963.calibrate(count=256, delay=200)
    
    #Calibracio Giroscopi
    mpu6500 = MPU6500(i2c_gy91)
    offsetGyro = mpu6500.calibrate(count=256, delay=0)
    
    #Guardar dades de calibracio
    logCal = open('/sd/cal.py', 'w')
    logCal.write('offsetMag = {} \nscaleMag = {} \noffsetGyro = {}'.format(offsetMag,scaleMag,offsetGyro))
    logCal.close()
    
    MPUCalibrat = True
    return offsetMag, scaleMag, offsetGyro
    

#Obtencio angular mitjançant Fuse


def Fuse():
    senMPU()
    fuse = Fusion()

    Timing = True
    Calibrate = True
    
    def getmag():
        senMPU()
        return mag
    
    if Calibrate:
        print("Calibrant. Presionar KEY un cop acabat.")
        fuse.calibrate(getmag, sw, lambda : pyb.delay(100))
        print(fuse.magbias)
    
    if Timing:
        '''
        mag = mpu.magnetic # Don't include blocking read in time
        accel = mpu.acceleration # or i2c
        gyro = mpu.gyro
        '''
        senMPU()
        start = time.ticks_us()  # Measure computation time only
        fuse.update(accel, gyro, mag, 0.005) # 1.97mS on Pyboard
        t = time.ticks_diff(time.ticks_us(), start)
        print("Update time (uS):", t)

    count = 0
    while True:
        senMPU()
        fuse.update(accel, gyro, mag, 0.005) # Note blocking mag read
        if count % 50 == 0:
            print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
        time.sleep_ms(10)
        count += 1


def senGPS():
    for i in range(50):
        try:
            line = uart_gps.readline().decode("utf-8")
            for x in line:
                my_gps.update(x)
        except:
           pass 
        time.sleep_ms(10)
        

    
