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
from math import log,floor
from fusion import Fusion
from pyb import Switch
from orientate import orientate
from micropyGPS import MicropyGPS

try:
    from cal import *
except:
    MPUCalibrat = False
else:
    MPUCalibrat = True

#from micropyGPS import MicropyGPS
global offsetMag, scaleMag, sw, my_gps, logname, etapactual
logname = "0000/00/00_00:00"
i2c_gy91=I2C(sda= pin.PB7, scl=pin.PB6)
uart_gps = UART(2, baudrate=9600, bits=8, parity=None, stop=1)
my_gps = MicropyGPS()
sw = Switch()
SF_DEG_S = 1


    
class color:
    GREEN   = '\033[92m'
    ORG     = '\033[33m'
    RED     = '\033[91m'
    FLICK   = '\033[5m'
    BOLD    = '\033[1m'
    END     = '\033[0m'
    
class msg:
    OK  = color.BOLD + color.GREEN + "OK" + color.END + color.END
    NOK = color.BOLD + color.ORG + "N-OK" + color.END + color.END
    ERR = color.BOLD + color.FLICK + color.RED +"ERROR CRITIC" + color.END + color.END + color.END
    
start = pyb.millis()




def auto():
    global etapactual
    test()
    orientacio.setup()
    cancel = 0
    entrada = str(input("Iniciar protocol d'enlairament? SI(Y) | NO(N): "))
    
    if entrada == "Y":
        mission_start=time.ticks_ms()
        while cancel != 1:
            entrada = str(input("Coloca el coet en posició de enlairament, envia (Y) quan estigui llest, (N) per cancelar: "))
            llest = 0
            etapactual = "Espera"
            altitudANT = altitud
            taltANT = talt
            while llest != 1:
                if entrada == "Y":
                    llest = 1
                    
                elif entrada == "N":
                    cancel = 1
                    llest = 1
                else:
                    entrada = str(input("Si us plau, escriu (Y) o (N)"))
            print("Estabilitzant mesures...",end="")
            for i in range(1000):
                hdg,ptch,roll = orientacio.calc()
            print(msg.OK)
            print("AoA = {}".format(90+orientacio.calc()[1]))
            mesurar(logname)
            mesures = 1
            while mesures == 1:
                calcVelVert()
                detectorEtapa()
                datalog(logname,etapactual,mission_start)
            
            
            
            
        












def test():
    print(color.BOLD + "Iniciant test del sistema\n" + color.END)
    print("Comprovant sd...",end="")
    if 'sd' in os.listdir('/'):
        print(msg.OK)
    else:
        print(msg.ERR)
        print(color.RED + "Prem Ctrl+D, si no funciona..." + color.END)
        print(color.RED + "verifica l'estat de la SD" + color.END)
    
    print("Comprovant estat del MPU9250...",end="")
    if 12 in i2c_gy91.scan() or 118 in i2c_gy91.scan():
        print(msg.OK)
    else:
        print(msg.ERR)
        print(color.RED + "Prem Ctrl+D, si no funciona..." + color.END)
        print(color.RED + "desconecta i reconecta la placa, sino revisa la tensió d'entrada a placa (5V) i les sortides (5V i 3v3)" + color.END)
        
    print("Comprovant estat del BMP280...",end="")
    if 104 in i2c_gy91.scan():
        print(msg.OK)
        senBMP()
        print("Altitud actual: {alt}".format(alt=altitud))
    else:
        print(msg.ERR)
        print(color.RED + "Prem Ctrl+D, si no funciona..." + color.END)
        print(color.RED + "desconecta i reconecta la placa, sino revisa la tensió d'entrada a placa (5V) i les sortides (5V i 3v3)" + color.END)
       
    print("Comprovant calibracio del sensor MPU9250...",end="")   
    try:
        print("Offset magnetometre: {}".format(offsetMag))
        print("Escala magnetometre: {}".format(scaleMag))
        print("Offset giroscopi: {}".format(offsetGyro))
    except:
        print(msg.nok)
        print(color.ORG + "No es disposa de dades de calibracio, executa calMPU()" + color.END)
        MPUCalibrat = False
    else:
        print(msg.OK)
        print(color.GREEN + "Dades de calibracio trobades, en cas de malfuncionament recalibrar amb calMPU()" + color.END)
        MPUCalibrat = True
        
    print("Comprovant conexió al sensor GPS...",end="")
    try:
        "" in uart_gps.readline()
    except:
        print(msg.ERR)
        print(color.RED + "Revisa la connexió del sensor" + color.END)
    else:
        print(msg.OK)
    
    for i in range(6):
        print("Comprovant cobertura GPS...",end="")
        try:
            #my_gps.local_offset = input("Introduir zona horaria: ")
            senGPS(500)
        except:
            print(msg.ERR)
            print(color.RED + "Ha succeit un error desconegut amb el modul senGPS" + color.END)
            break
        else:
            if my_gps.satellite_data_updated() and my_gps.satellites_in_use > 0:
                timezone = 2
                dt=(int("20"+str(my_gps.date[2])),my_gps.date[1],my_gps.date[0],1,my_gps.timestamp[0]+timezone,my_gps.timestamp[1],floor(my_gps.timestamp[2]),0)
                pyb.RTC().datetime(dt)
                print(msg.OK)
                print(color.GREEN + "Hi ha senyal GPS" + color.END)
                
                print("S'han detectat {SATS} satel·lits".format(SATS=my_gps.satellites_in_use))
                RealT="{hora}:{minut}".format(hora=my_gps.timestamp[0], minut=my_gps.timestamp[1])
                print("Avui hauria de ser {DATE} i la hora actual hauria de ser {TIME}".format(DATE=my_gps.date, TIME=RealT))
                global logname
                logname="20{y}_{m}_{d}_{h}_{mn}".format(y=my_gps.date[2],m=my_gps.date[1],d=my_gps.date[0],h=my_gps.timestamp[0]+timezone,mn=my_gps.timestamp[1])
                print("Les coordenades actuals del coet són: {lat} {lon}".format(lat=my_gps.latitude,lon=my_gps.longitude))
                break
            elif i < 5:
                print(msg.NOK)
                print(color.ORG + "No hi ha cobertura GPS, reintentant..." + color.END)
            
            elif i == 5:
                print(msg.ERR)
                print(color.RED + "No s'ha pogut establir cobertura GPS, asegura que el coet es troba a l'exterior i executa test()" + color.END)
            
        
def mesurar(nom):
    log = open('/sd/log{}.csv'.format(nom), 'w')
    log.write('temps (s), etapa, latitud, longitud, pressio (Pa), altitud (m), AoA(º), Velocitat(m/s), Velocitat_GPS(m/s), acX(m/s²), acY(m/s²), acZ(m/s²)\n')
    log.close()
    
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
def datalog(nom,lastevent,mission_start):
    log = open('/sd/log{}.csv'.format(nom), 'a')
    senBMP()
    senMPU()
    senGPS(2)
    orientacio.calc()
    t = pyb.elapsed_millis(mission_start)
    line = '{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(t,lastevent,str(my_gps.latitude).replace(",","").replace("'",""),str(my_gps.longitude).replace(",","").replace("'",""),p,altitud,90+orientacio.calc()[1],VelVert,my_gps.speed[2]/3.6,acX,acY,acZ,)
    print(line)
    log.write(line)
    log.close()
    
#Obtencio de dades de pressio
def senBMP():
    bmp = bmp280.BMP280(i2c_gy91)
    global p, altitud, talt
    p = bmp.pressure
    altitud = (log(101325/p))/0.00012
    talt = time.ticks_ms()
    
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
    

#Obtencio angular mitjançant Fusion

class orientacio():
    def setup():
        senMPU()
        global ori
        ori = Fusion()

        Timing = True
        Calibrate = False
        
        def getmag():
            senMPU()
            return mag
        
        if Calibrate:
            print("Calibrant. Presionar KEY un cop acabat.")
            ori.calibrate(getmag, sw, lambda : pyb.delay(100))
            print(ori.magbias)
        
        if Timing:
            '''
            mag = mpu.magnetic # Don't include blocking read in time
            accel = mpu.acceleration # or i2c
            gyro = mpu.gyro
            '''
            senMPU()
            start = time.ticks_us()  # Measure computation time only
            ori.update(accel, gyro, mag, 0.005) # 1.97mS on Pyboard
            t = time.ticks_diff(time.ticks_us(), start)
            print("Temps de mostreig (uS):", t)
    
    def calc():
        count = 0
        while True:
            senMPU()
            ori.update(accel, gyro, mag, 0.005) # Note blocking mag read
            if count % 150 == 0:
                return(ori.heading, ori.pitch, ori.roll)
                break
            time.sleep_ms(1)
            count += 1


def senGPS(times):
    for i in range(times):
        try:
            line = uart_gps.readline().decode("utf-8")
            for x in line:
                my_gps.update(x)
        except:
           pass 
        time.sleep_ms(15)
        
def detectorEtapa():
    global etapactual, tempsAp
    etapes=["Espera", "Acceleracio", "Coasting", "Ap", "Descens", "Touchdown"]
    if etapactual == "Espera":
        if acX <= 7: etapactual = "Acceleracio"
    elif etapactual == "Acceleracio":
        if acX >= 1: etapactual = "Coasting"
    elif etapactual == "Coasting":
        if VelVert <= 1: 
            tempsAp = time.ticks_ms()
            etapactual = "Ap"
    elif etapactual == "Ap":
        if pyb.elapsed_millis(tempsAp) > 1000:
            etapactual = "Descens"
    elif etapactual == "Descens":
        if VelVert <= 0.3: etapactual = "Touchdown"
            
def calcVelVert():
    global VelVert, altitudANT, taltANT
    try:
        taltANT
        altitudANT
    except:
        taltANT = talt
        altitudANT = altitud
    try:
        VelVert = (altitud-altitudANT)/(talt/1000-taltANT/1000) #Calcul de la velocitat vertical en m/s
    except:
        VelVert = 0
    if -0.4<=VelVert<=0.4:
        VelVert = 0
    
    altitudANT = altitud
    taltANT = talt
