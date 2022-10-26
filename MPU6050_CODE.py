from imu import MPU6050
from time import sleep
from machine import Pin, SoftI2C
import math
import time


i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6050(i2c)
earths_gravitation = 9.8

def accel_mps(g):
    mps = g * earths_gravitation
    return mps

distX = 0
distY = 0
distZ = 0

accAX = 0
accAY = 0

gyx = 0
gyy = 0
gyz = 0

st = time.time_ns()

while True:
    
    gx = round(imu.gyro.x)
    gy = round(imu.gyro.y)
    gz = round(imu.gyro.z)
    
    tem = round(imu.temperature,2)

    ax = round(imu.accel.x,2)
    ay = round(imu.accel.y,2)
    az = round(imu.accel.z,2)
    
    et = time.time_ns()
    dt = ((et - st)/1000000)/1000
    
   
    distX = distX + ax
    dx = distX - ax
    x = dx/dt
    
    distY = distY + ay
    dy = distY - ay
    y = dy/dt
    
    distZ = distZ + az
    dz = distZ - az
    z = dz/dt
    
    accAngleX = (math.atan2(accel_mps(ay) , math.sqrt(math.pow(accel_mps(ax),2) + math.pow(accel_mps(az),2))) * 180 / math.pi) 
    accAngleY = (-1 * math.atan2(accel_mps(ax) , math.sqrt(math.pow(accel_mps(ay),2) + math.pow(accel_mps(az),2))) * 180 / math.pi)  
    
    
    gyx = gyx + gx * dt
    gyy = gyy + gy * dt
    gyz = gyz + gz * dt
    
   
    accAX = 0.94 * accAX + 0.06 * accAngleX
    print("up:",round(accAX))

    
    accAY = 0.94 * accAY + 0.06 * accAngleY
    print("up:",round(accAY))
    print('\n')
    
    
    print(dt)
    sleep(0.1)
