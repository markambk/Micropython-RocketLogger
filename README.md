# RocketLogger
A simple micropython program that uses libraries to get useful info from a model rocket, (AoA, altitude, coordinates, acceleration, speed). It also detects Apoapsis (Ap) and deploys a piston system parachute.

HARDWARE USED:

  - WeAct STM32F411 with Micropython Port and Winbond W25Q128 memory.
    - How to Flash: https://github.com/WeActTC/WeAct_F411CE-MicroPython
    - Pinout: https://github.com/WeActTC/MiniF4-STM32F4x1
    - Useful info: https://github.com/mcauser/WEACT_F411CEU6
    - Example Code + Block Diagram: https://www.weact-tc.cn/2020/01/01/micropython/ 

  - GY-91 10DOF IMU (From Aliexpress) (BMP280 + MPU9250) connected on I2C

  - SPI Micro SDCard Adapter

  - GY-GPS6MV2 [CONNECT AT 3v3! NOT 5V!] ( Ublox NEO-6M)

  - 3DR 915Mhz 100mW UART antenna.

LIBRARIES USED:
  - micropython-MPU9250: https://github.com/tuupola/micropython-mpu9250
  - ImuMag-Fusion: https://github.com/wystephen/ImuMag-fusion
  - micropython-bmp280: https://github.com/dafvid/micropython-bmp280
  - micropyGPS: https://github.com/inmcm/micropyGPS
