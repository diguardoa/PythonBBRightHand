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

#Motor DIR
GPIO.setup("P8_26", GPIO.OUT)
GPIO.setup("P8_14", GPIO.OUT)
GPIO.setup("P8_15", GPIO.OUT)
GPIO.setup("P8_16", GPIO.OUT)
GPIO.setup("P8_17", GPIO.OUT)
#Motor PWM
PWM.start("P9_31", 0.0)
PWM.set_frequency("P9_31", 1000.0) 
PWM.start("P9_14", 0.0)
PWM.set_frequency("P9_14", 1000.0) 
PWM.start("P8_19", 0.0)
PWM.set_frequency("P8_19", 1000.0) 
PWM.start("P9_42", 0.0)
PWM.set_frequency("P9_42", 1000.0) 
PWM.start("P9_28", 0.0)
PWM.set_frequency("P9_28", 1000.0) 


#User Led 5
GPIO.setup("USR0", GPIO.OUT)

counter = 0
status  = 0
pwm_val = 0.0;
dir_val = 0.0;

print "Toggle started"

while True:

    if status == 0:
        pwm_val = 0.0
        dir_val = 0.0
        
    if status == 1:
        pwm_val = 30.0
        dir_val = 0.0  
        
    if status == 2:
        pwm_val = 30.0
        dir_val = 1.0        
        
    if dir_val > 0.0:     
        GPIO.output("USR0", GPIO.LOW)
        GPIO.output("P8_26", GPIO.LOW)
        GPIO.output("P8_14", GPIO.LOW)
        GPIO.output("P8_15", GPIO.LOW)
        GPIO.output("P8_16", GPIO.LOW)
        GPIO.output("P8_17", GPIO.LOW)
    else:
        GPIO.output("USR0", GPIO.HIGH)
        GPIO.output("P8_26", GPIO.HIGH)
        GPIO.output("P8_14", GPIO.HIGH)
        GPIO.output("P8_15", GPIO.HIGH)
        GPIO.output("P8_16", GPIO.HIGH)
        GPIO.output("P8_17", GPIO.HIGH)
        
    PWM.set_duty_cycle("P9_31", pwm_val)
    PWM.set_duty_cycle("P9_14", pwm_val)
    PWM.set_duty_cycle("P8_19", pwm_val)
    PWM.set_duty_cycle("P9_42", pwm_val)
    PWM.set_duty_cycle("P9_28", pwm_val)       
        
    time.sleep(0.01)    
    counter = counter+1
    if counter > 100:
        counter = 0
        status = status+1
        if status > 2:
            status = 0
        print status, pwm_val, dir_val    
        


