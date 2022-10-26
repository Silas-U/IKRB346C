from time import sleep
from machine import Pin
from machine import PWM

pwm = PWM(Pin(2))
pwm.freq(50)

def setServoCycle(position):
    pwm.duty_u16(position)
    sleep(0.01)
    
def angle(a):
    deg = a * 50
    return deg

init_pos = angle(10)
final_pos = angle(180)

for pos in range(init_pos, final_pos, 30):
    setServoCycle(pos)