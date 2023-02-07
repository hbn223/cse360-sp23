#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import time
from Motor import *
from Led import *
from Buzzer import *

PWM=Motor()
led=Led()
buzzer=Buzzer()

if __name__ == '__main__':
    try:
        PWM.setMotorModel(1000,1000,1000,1000)
        time.sleep(2)
        PWM.setMotorModel(2200,2200,-1500,-1500)
        time.sleep(1)
        led.ledIndex(0x01,255,0,0)

        PWM.setMotorModel(1000,1000,1000,1000)
        time.sleep(2)
        PWM.setMotorModel(2200,2200,-1500,-1500)
        time.sleep(1)
        led.ledIndex(0x02,0,0,255)

        PWM.setMotorModel(1000,1000,1000,1000)
        time.sleep(2)
        PWM.setMotorModel(2200,2200,-1500,-1500)
        time.sleep(1)
        led.ledIndex(0x04,0,255,0)

        PWM.setMotorModel(1000,1000,1000,1000)
        time.sleep(2)
        PWM.setMotorModel(2200,2200,-1500,-1500)
        time.sleep(1)
        led.ledIndex(0x08,255,255,0)
        PWM.setMotorModel(0,0,0,0)

        buzzer.run('1')
        time.sleep(1)
        buzzer.run('0')

    except KeyboardInterrupt:
        PWM.setMotorModel(0,0,0,0)
        led.colorWipe(led.strip, Color(0,0,0))
        buzzer.run('0')
        destroy()

