import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

import time
import math


#MODIFICA SCHEDA
#Routing:
#Motor 1:
#PWM P9_31 ok
#DIR P8_26 ok
#Motor 2:
#PWM P9_29 -> troncato -> ponte connesso a P9_14
#DIR P8_25 -> troncato -> ponte connesso a P8_14
#Motor 3:
#PWM P8_19 ok
#DIR P8_24 -> troncato -> ponte connesso a P8_15
#Motor 4:
#PWM P8_13 -> troncato -> ponte connesso a P9_42
#DIR P8_23 -> troncato -> ponte connesso a P8_16
#Motor 5:
#PWM P9_28 ok
#DIR P8_22 -> troncato -> ponte connesso a P8_17
#Motor 6:
#PWM P9_42(da aggiungere jumper) ok
#DIR P8_21 -> troncato -> ponte connesso a P8_18

#Motor1
PWM.start("P9_31", 0.0, 1000.0, 1.0)
PWM.start("P9_14", 0.0, 1000.0, 1.0)
PWM.start("P8_19", 0.0, 1000.0, 1.0)
PWM.start("P9_42", 0.0, 1000.0, 1.0)
PWM.start("P9_28", 0.0, 1000.0, 1.0)

#User Led 5
GPIO.setup("USR0", GPIO.OUT)


counter = 0
toggle  = 0
alpha = 0.0
pwm_value = 0.0
alpha = 0.0

print "Toggle started"

while True:

    if toggle > 0:
        
        GPIO.output("USR0", GPIO.LOW)
        PWM.set_duty_cycle("P9_31", 0.0, 1000.0, 1.0)
        PWM.set_duty_cycle("P9_14", 0.0, 1000.0, 1.0)
        PWM.set_duty_cycle("P8_19", 0.0, 1000.0, 1.0)
        PWM.set_duty_cycle("P9_42", 0.0, 1000.0, 1.0)
        PWM.set_duty_cycle("P9_28", 0.0, 1000.0, 1.0)
        
    else:
       
        GPIO.output("USR0", GPIO.LOW)
        PWM.set_duty_cycle("P9_31", 0.0, 1000.0, 99.0)
        PWM.set_duty_cycle("P9_14", 0.0, 1000.0, 99.0)
        PWM.set_duty_cycle("P8_19", 0.0, 1000.0, 99.0)
        PWM.set_duty_cycle("P9_42", 0.0, 1000.0, 99.0)
        PWM.set_duty_cycle("P9_28", 0.0, 1000.0, 99.0)
        
        
    time.sleep(0.01)    
    counter = counter+1
    if counter > 100:
        counter = 0
        toggle = 1-toggle
        print toggle
        


