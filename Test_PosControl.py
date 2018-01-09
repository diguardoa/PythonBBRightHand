import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_I2C import Adafruit_I2C
import socket
import struct
import time

import math

data_in_str = "abcdabcdabcdabcdabcdabcdabcdabcd"
data_in_float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
data_out_float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
data_out_str = "abcdabcdabcdabcdabcdabcdabcdabcd"

UDP_PORT_IN = 12000
UDP_PORT_OUT = 12001
UDP_IP_OUT = "10.100.34.247"

sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.setblocking(False)
sock_in.bind(("0.0.0.0", UDP_PORT_IN))

sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Code on Teensy:
#uint8_t adc_pins[] = {A0,A1,A2,A3, A6,A7,A8,A9,A10,A11,A13,A14, A18,A19,A20};
adc_linpot_i = [11, 12, 0, 1, 2]
adc_strain_i = [7, 6, 5, 4, 3]
adc_jointpot_i = [13, 14, 10, 9, 8]
index = 0;

adc_linpot = [0, 0, 0, 0, 0]
adc_strain = [0, 0, 0, 0, 0]
adc_jointpot = [0, 0, 0, 0, 0]

adc_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

i2c = Adafruit_I2C(9) #communication with Teensy at address 9

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


counter = 0
toggle  = 0
status = 0
alpha = 0.0
pwm_value = [0.0, 0.0, 0.0, 0.0, 0.0]
alpha = 0.0
ref = [500.0, 500.0, 500.0, 500.0, 500.0]
err = [0.0, 0.0, 0.0, 0.0, 0.0]
pos = [500.0, 500.0, 500.0, 500.0, 500.0]

print "Handexos started"

while True:
    
    
    # UDP COMMUNICATION
    try:
        data_in_str, addr = sock_in.recvfrom(32) 
    except:
        b = 1
    else:
        data_in_float = struct.unpack(">8f", data_in_str)

        data_out_float[0] = adc_BB[0]
        
        data_out_str = struct.pack(">8f", data_out_float[0], data_out_float[1], data_out_float[2], data_out_float[3], data_out_float[4], data_out_float[5], data_out_float[6], data_out_float[7])
        sock_out.sendto(data_out_str, (UDP_IP_OUT, UDP_PORT_OUT))
    
    
    # ADC READING 
    adc_values = struct.unpack("<16h", bytes(bytearray(i2c.readList(0, 32))))
    for i in range(0, 5):
        index = adc_linpot_i[i]
        adc_linpot[i] = adc_values[index]
        index = adc_strain_i[i]
        adc_strain[i] = adc_values[index]
        index = adc_jointpot_i[i]
        adc_jointpot[i] = adc_values[index]
        
    
    if status==0:
        for i in range(0, 5):
            pwm_value[i] = 0.0
    if status==1:
        for i in range(0, 5):
            ref[i] = 750.0
    if status==2:
        for i in range(0, 5):
            ref[i] = 1000.0

    for i in range(0, 5):
        pos[i] = adc_linpot[i]

    
    if status > 0:
        for i in range(0, 5):
            err[i] = (ref[i]-pos[i])*0.02
            if err[i] > 1.0:
                err[i] = 1.0
            if err[i] < -1.0:
                err[i] = -1.0
            pwm_value[i] = err[i]*40.0
    else:
        for i in range(0, 5): #disable motors
            pwm_value[i] = 0.0
        
    #motor1
    PWM.set_duty_cycle("P9_31", abs(pwm_value[0])) #duty cycle
    if pwm_value[0] < 0:
        GPIO.output("P8_26", GPIO.LOW) #direction
    else:
        GPIO.output("P8_26", GPIO.HIGH)        

    #motor2
    PWM.set_duty_cycle("P9_14", abs(pwm_value[1]))
    if pwm_value[1] < 0:
        GPIO.output("P8_14", GPIO.LOW)
    else:
        GPIO.output("P8_14", GPIO.HIGH)   
        
    #motor3
    PWM.set_duty_cycle("P8_19", abs(pwm_value[2]))
    if pwm_value[2] < 0:
        GPIO.output("P8_15", GPIO.LOW)
    else:
        GPIO.output("P8_15", GPIO.HIGH)   
        
    #motor4
    PWM.set_duty_cycle("P9_42", abs(pwm_value[3]))
    if pwm_value[3] < 0:
        GPIO.output("P8_16", GPIO.LOW)
    else:
        GPIO.output("P8_16", GPIO.HIGH)   

    #motor5
    PWM.set_duty_cycle("P9_28", abs(pwm_value[4])) 
    if pwm_value[4] < 0:
        GPIO.output("P8_17", GPIO.LOW)
    else:
        GPIO.output("P8_17", GPIO.HIGH)   

    
     
    
    time.sleep(0.01)    
    counter = counter+1
    if counter > 100:
        counter = 0
        print ref, pos, err, pwm_value
        toggle = 1-toggle
        status = status+1
        if status > 2:
            status = 0
        
        


    
#ADCcapture.stop()
#ADCcapture.wait()
#ADCcapture.close()