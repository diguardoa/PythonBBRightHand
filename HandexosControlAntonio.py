import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from Adafruit_I2C import Adafruit_I2C
import socket
import struct
import time
import math
#######################################################################
# VARIABILI ANTONIO
# passare da ADC in forza misurata in Newton (chiedi a Daniele)
# Metti a posto il loop periodico con le funzioni del tempo (vedi se daniele si ricorda. ad ogni modo serve avere il dispositivo davanti per fare altre considerazioni)
t_control = 0.01 # [s]
k_adc_to_newton = 1
pwm_frequency = 20000.0 # [Hz] il default e' 1000.0

# Init UDP connection
UDP_PORT_IN = 12000
UDP_IP_IN = "10.100.39.253" # From Proxy
UDP_PORT_OUT = 12001
UDP_IP_OUT = "10.100.39.253" # To Proxy

pkt_from_proxy = 0 # il numero del pacchetto elaborato dal proxy (per controllare se c'e' ritardo)
pkt = 0 # numero di pacchetto mandato al proxy (corrente)

# primo ciclo del programma. Serve per compensare la gravita'
first = 1 


# guadagno proporzionale anello forza (dipende dalla posizione del motore)
kp_force = [60, 60, 60, 60, 60] 
kp_force_low = [20, 20, 20, 20, 20]
kp_force_high = [60, 60, 60, 60, 60]
pos_threshold = [700, 700, 700, 700, 700]

kv = [0, 0, 0, 0, 0] # vuole valori positivi, solo per mode 9


# direzione del sensore di forza (nota: l'indice e' montato al contrario)
direction_sensor = [1, -1, 1, 1, 1]
####################################################################### 
####################################################################### 
#                    VARIABILI CONTROLLO
####################################################################### 
#######################################################################
mode = 0.0 # disable = 0.0, enable = 1.0, reading_only = 2.0, pos_sine_wave 4.0, transparency 5.0
            # 6.0 High stiffness #7.0 Low stiffness # 8.0 sharp wall # 9.0 force + velocity (componente viscosa)

angle = 0.0;
angle_step = 0.01;
C1 = 12/1551;
C2 = 53.37/12;
#Riferimenti (sovrascritti da pacchetti UDP in ingresso)
safe_timer = 0;

status = 0.0 #non utilizzato
ref_force = [0.0, 0.0, 0.0, 0.0, 0.0]
ref_pos = [700.0, 700.0, 700.0, 700.0, 710.0]

#abilitazione singole dita:
enable = [0.0, 1.0, 0.0, 0.0, 0.0] #1.0 abilitato, altro valore, disabilitato

#guadagno feedforward (pwm = force_ref*kp_ffwd_force)
kp_ffwd_force = 1.0;



#offset sensori di forza 
strain_offset = [0, 0, 0, 0, 0] #[-303.0, -328.0, -375.0, -324.0, -338.0]  #[-288.0, -335.0, -100.0, 0.0, 0.0]

#offset posizione attuatori lineari (usare per regolare la posizione di anulare e mignolo rispetto al medio)
ref_pos_offset = [0.0, 0.0, 0.0, 0.0, 0.0]

#limiti di corsa degli attuatori lineari
ref_pos_max = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
ref_pos_min = [5.0, 5.0, 5.0, 5.0, 5.0]

#guadagno proporzionale anello posizione (usato su anulare e mignolo per seguire la posizione del medio)
kp_pos = [1.2, 1.2, 1.2, 1.2, 1.2]

#massimo valore pwm (range 0-100)
pwm_max = [90.0, 90.0, 90.0, 90.0, 90.0]



#settaggio controllo impedenza
en_imp = 0.0
position_thr = [500.0, 500.0, 500.0, 1024.0, 1024.0]
compenetration = [0.0, 0.0, 0.0, 0.0, 0.0]
stiffness = [0.25, 0.25, 0.25, 0.25, 0.25]
 
wall_pos = 0
wall_depth = 0
wall_stiff = 0
        
 
####################################################################### 
####################################################################### 
 #                    INITIALIZATION
####################################################################### 
#######################################################################
puppa = 0
buff = 100
j = 0
n =0
pos = [0.0, 0.0, 0.0, 0.0, 0.0] # ok
pos_1 = [0.0, 0.0, 0.0, 0.0, 0.0]
force = [0.0, 0.0, 0.0, 0.0, 0.0]
force_rectified = [0.0, 0.0, 0.0, 0.0, 0.0]
filtered_force = [0.0, 0.0, 0.0, 0.0, 0.0]
force_buffer = [[0.0 for x in range(5)] for y in range(buff)]
joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0] # ok
err_force = [0.0, 0.0, 0.0, 0.0, 0.0]
err_pos = [0.0, 0.0, 0.0, 0.0, 0.0] 
pwm_value = [0.0, 0.0, 0.0, 0.0, 0.0]
counter = 0
    

## Init Socket UDP e Pacchetti

# 12 byte: pkt, mode, 5 ref force, 5 ref pos
data_in_float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 22 byte: pkt, mode, 5 force sens, 5 linpot, 5 jointpot, 5 thumb jointpot
data_out_float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# Variabili di appoggio per la costruzione dei pacchetti (ricevo tutto come stringhe)
data_in_str = "" 
data_out_str = "" 

sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_in.setblocking(False)
sock_in.bind((UDP_IP_IN, UDP_PORT_IN))
sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

## Code on Teensy (MODIFICATO ANTONIO 1/1/18)
# uint8_t adc_pins[] = {A0,A1,A2,A3,A6,A7,A8,A9,A10,A11,A14,A13,A18,A19,A20,A15, A16, A17, A21, A22};
# I valori trasmessi sono 20
# 0 = A0
# 1 = A1
# 2 = A2
# 3 = forza mignolo
# 4 = forza indice (verso opposto)
# 5 = forza medio
# 6 = forza anulare
# 7 = forza pollice
# 8 = rotoidale indice
# 9 = rotoidale pollice
# 10 = rotoidale medio
# 11 = A13
# 12 = A18: 325
# 13 = A19: PIN LIBERO: Filo Verdino (Mignolo?)
# 14 = A20: rotoidale spread
# 15 = Rosso, A15 [POLLICE]
# 16 = Nero, A16 [POLLICE]
# 17 = Bianco, A17 [POLLICE]
# 18 = A21: PIN LIBERO: Filo Nero (Anulare?)
# 19 = Grigio, [POLLICE]

# linpot e strain: nell'ordine Thumb, Index, Middle, Ring, Little
adc_linpot_i = [11, 12, 0, 1, 2]
adc_strain_i = [7, 4, 5, 6, 3]

# joinpot: nell'ordine Spread, Index, Middle, Ring, Pinky
adc_jointpot_i = [14, 8, 10, 18, 13]
# joinpot_thumb
adc_jointpot_thumb_i = [9, 15, 16, 17, 19]

# Variabile di supporto (non c'entra con il dito indice)
index = 0;

# Array vuoti (inizializzazione)
adc_linpot = [0, 0, 0, 0, 0]
adc_strain = [0, 0, 0, 0, 0]
adc_jointpot = [0, 0, 0, 0, 0]
adc_jointpot_thumb = [ 0, 0, 0, 0, 0]

adc_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

i2c_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

i2c = Adafruit_I2C(9) #communication with Teensy at address 9



## COONFIGURAZIONI DEI MOTORI
#Motor DIR
GPIO.setup("P8_26", GPIO.OUT)
GPIO.setup("P8_14", GPIO.OUT)
GPIO.setup("P8_15", GPIO.OUT)
GPIO.setup("P8_16", GPIO.OUT)
GPIO.setup("P8_17", GPIO.OUT)
#Motor PWM
PWM.start("P9_31", 0.0)
PWM.set_frequency("P9_31", pwm_frequency) 
PWM.start("P9_14", 0.0)
PWM.set_frequency("P9_14", pwm_frequency) 
PWM.start("P8_19", 0.0)
PWM.set_frequency("P8_19", pwm_frequency) 
PWM.start("P9_42", 0.0)
PWM.set_frequency("P9_42", pwm_frequency) 
PWM.start("P9_28", 0.0)
PWM.set_frequency("P9_28", pwm_frequency) 




print "Handexos started"

while True:
    

    
    ####################################################################### 
    ####################################################################### 
    #                    UDP COMMUNICATION
    ####################################################################### 
    #######################################################################
    
    

    # N di pacchetto 
    pkt = pkt + 1

    # dopo 100 pacchetti non ricevuti dal proxy si mette lo stato a 0 (quindi i motori si bloccano)
    # Commenta se non piu' necessario
    if (safe_timer > 0):
        safe_timer = safe_timer -1
    else:
        status = 0.0
    
    # Vedo se ho pachetti in arrivo dal proxy
    try:   
        data_in_str, addr = sock_in.recvfrom(48) 
    except:
        b = 1
    else:     
        data_in_float = struct.unpack("<12f", data_in_str)
            
        safe_timer = 100
        pkt_from_proxy = data_in_float[0]
        mode = data_in_float[1]
    
        ref_force[0] = data_in_float[2]
        ref_force[1] = data_in_float[3]
        ref_force[2] = data_in_float[4]
        ref_force[3] = data_in_float[5]
        ref_force[4] = data_in_float[6]

        ref_pos[0] = data_in_float[7]
        ref_pos[1] = data_in_float[8]
        ref_pos[2] = data_in_float[9]
        ref_pos[3] = data_in_float[10]
        ref_pos[4] = data_in_float[11]
        
        
    # Mando pacchetto al proxy (float da 4 byte Big Endian). NB: il primo pacchetto e' fatto da tutti zero (tranne il primo float pari a 1)
    data_out_float[0] = pkt;
    data_out_float[1] = mode;
    #forces
    data_out_float[2] = force[0]#filtered_force
    data_out_float[3] = force[1]#force_rectified
    data_out_float[4] = force[2]
    data_out_float[5] = force[3]
    data_out_float[6] = force[4]
    #position
    data_out_float[7] = adc_linpot[0]
    data_out_float[8] = adc_linpot[1]
    data_out_float[9] = adc_linpot[2]
    data_out_float[10] = adc_linpot[3]
    data_out_float[11] = adc_linpot[4]
    
    #data rotative potentiometers
    data_out_float[12] = adc_jointpot[0]
    data_out_float[13] = adc_jointpot[1]
    data_out_float[14] = adc_jointpot[2]
    data_out_float[15] = adc_jointpot[3]
    data_out_float[16] = adc_jointpot[4]

    #data rotative thumb potentiometers
	
	data_out_float[17] = adc_jointpot_thumb[0]
	data_out_float[18] = adc_jointpot_thumb[1]
	data_out_float[19] = adc_jointpot_thumb[2]
	data_out_float[20] = adc_jointpot_thumb[3]
	data_out_float[21] = adc_jointpot_thumb[4]

    data_out_str = struct.pack("<22f", data_out_float[0], data_out_float[1], data_out_float[2], data_out_float[3], data_out_float[4], data_out_float[5], data_out_float[6], data_out_float[7], data_out_float[8], data_out_float[9], data_out_float[10], data_out_float[11],data_out_float[12], data_out_float[13], data_out_float[14], data_out_float[15], data_out_float[16], data_out_float[17], data_out_float[18], data_out_float[19], data_out_float[20], data_out_float[21])
    sock_out.sendto(data_out_str, (UDP_IP_OUT, UDP_PORT_OUT))

    
    
    ####################################################################### 
    ####################################################################### 
    #                    ANALOG INPUT READING
    ####################################################################### 
    #######################################################################
    #
    
    try:
    	i2c_values = i2c.readList(0, 32)
    except:
    	print "No i2c packet is received"
    	# blocco i motori
        status = 0.0
	else:
	    if i2c_values!=-1:
	        adc_values = struct.unpack("<16h", bytes(bytearray(i2c.readList(0, 32))))
	        if j < buff-1:
	            j = j+1
	        for i in range(0, 5):
	            index = adc_linpot_i[i]
	            adc_linpot[i] = adc_values[index]

	            index = adc_strain_i[i]
	            adc_strain[i] = adc_values[index]

	            index = adc_jointpot_i[i]
	            adc_jointpot[i] = adc_values[index]

	            index = adc_jointpot_thumb_i[i]
	            adc_jointpot_thumb[i] = adc_values[index]

	            joint_pos[i] = adc_jointpot[i]
	            pos[i] = adc_linpot[i]
	            
	            # Parite inerente alla compensazione di gravita'. Funziona solo la prima volta che parte lo script
	            # Set the offset
	            if first == 1:
	                strain_offset[i] = adc_strain[i]
	            if i == 4:
	                first = 0

	            # Remove the offset of the sensor data    
	            force[i] = direction_sensor[i]*k_adc_to_newton * (adc_strain[i] - strain_offset[i])
            
           
        

    ####################################################################### 
    ####################################################################### 
    #                    CONTROL
    ####################################################################### 
    #######################################################################
    #   Rimasto lo stesso codice di Daniele, aggiunta solo una modalita' (per il controllo Bilaterale)

   

	# Modifica dei Gain di Forza a seconda della posizione degli attuatori (per ridurre instabilita')
	for i in range(0,5):
		if poi[i] > pos_threshold[i]:
			kp_force[i] = kp_force_low[i]
		else:
			kp_force[i] = kp_force_high[i]

		  
    #DEFAULT condition: disable motors
    for i in range(0, 5): #disable motors
            pwm_value[i] = 0.0
            
    #CONTROLLO FORZA FEEDFORWARD (NO SENSORI DI FORZA)
    if mode == 1.0:
        for i in range(0, 5):
            pwm_value[i] = ref_force[i]*kp_ffwd_force
            
    #CONTROLLO POSIZIONE (inteso sul motore)
    if mode == 2.0:
        for i in range(0, 5):
            err_pos[i] = ref_pos[i]-pos[i]
            pwm_value[i] = err_pos[i]*kp_pos[i]    
            
    # CONTROLLO DI FORZA 
    if mode == 3.0:
        
        #Primi tre attuatori in controllo di forza
        for i in range(0, 5):
            err_force[i] = ref_force[i]-force[i]
            pwm_value[i] = err_force[i]*kp_force[i]
           
    if mode == 4.0: #DEMO: sinsuoide di posizione
        status = 1.0
        angle = angle + angle_step;
        if angle>6.28:
            angle = 0.0
        for i in range(0, 5):
                ref_pos[i] = 500 + 400*math.sin(angle)
        for i in range(0, 5):
            err_pos[i] = ref_pos[i]-pos[i]
            pwm_value[i] = err_pos[i]*kp_pos[i]       

    if mode == 5.0: #DEMO: trasparenza con sensori di forza
        status = 1.0
        for i in range(0, 5):
            ref_force[i] = 0.0
            err_force[i] = ref_force[i]-force[i]
            pwm_value[i] = err_force[i]*kp_force[i]        
    
    if mode == 6.0: #DEMO: stiffness
        status = 1.0
        
        wall_pos = 50
        wall_depth = 450
        wall_stiff = 100
        
        for i in range(0, 5):
            
            if(pos[i]>wall_pos):
                if(pos[i]<(wall_pos+wall_depth)):
                    ref_force[i] = (pos[i]-wall_pos)*wall_stiff/wall_depth
                else:
                    ref_force[i] = wall_stiff
            else:
                ref_force[i] = 0.0
            
            err_force[i] = -ref_force[i]-force[i]
            pwm_value[i] = err_force[i]*kp_force[i]        
                    

    if mode == 7.0: #DEMO: stiffness
        status = 1.0
        
        wall_pos = 50
        wall_depth = 450
        wall_stiff = 25
        
        for i in range(0, 5):
            
            if(pos[i]>wall_pos):
                if(pos[i]<(wall_pos+wall_depth)):
                    ref_force[i] = (pos[i]-wall_pos)*wall_stiff/wall_depth
                else:
                    ref_force[i] = wall_stiff
            else:
                ref_force[i] = 0.0
            
            err_force[i] = -ref_force[i]-force[i]
            pwm_value[i] = err_force[i]*kp_force[i]        

    if mode == 8.0: #DEMO: stiffness
        status = 1.0
        
        wall_pos = 350
        wall_depth = 200
        wall_stiff = 100
        
        for i in range(0, 5):
            
            if(pos[i]>wall_pos):
                if(pos[i]<(wall_pos+wall_depth)):
                    ref_force[i] = (pos[i]-wall_pos)*wall_stiff/wall_depth
                else:
                    ref_force[i] = wall_stiff
            else:
                ref_force[i] = 0.0
            
            err_force[i] = -ref_force[i]-force[i]
            pwm_value[i] = err_force[i]*kp_force[i] 

    # CONTROLLO DI FORZA con limitazione della velocita'
    if mode == 9.0:
        
        #Primi tre attuatori in controllo di forza
        for i in range(0, 5):
            vel = t_control * (pos[i] - pos_1[i])
            pos_1[i] = pos[i]
            err_force[i] = ref_force[i]-force[i]
            pwm_value[i] = err_force[i]*kp_force[i] - kv[i]*vel

    # POSITION LIMITS
    for i in range(0, 5):
        if(pos[i]>ref_pos_max[i]):
            if (pwm_value[i]>0):
                pwm_value[i] = 0
        if(pos[i]<ref_pos_min[i]):
            if (pwm_value[i]<0):
                pwm_value[i] = 0
            
            
    # PWM output SATURATION        
    for i in range(0, 5):
        if (pwm_value[i]>pwm_max[i]):
            pwm_value[i] = pwm_max[i]  
        if (pwm_value[i]<-pwm_max[i]):
            pwm_value[i] = -pwm_max[i]      
       
       
    # ENABLE FINGERS   
    for i in range(0, 5):
        if (enable[i]!=1.0):
            pwm_value[i] = 0.0    
            
    # CHECK SYSTEM STATUS
    if(status != 1.0):
        for i in range(0, 5): #disable motors
            pwm_value[i] = 0.0
    
    # Rendering palla di gomma
    #if (en_imp==1.0):
    #    for i in range(0, 3):
    #        compenetration[i] = pos[i]-position_thr[i]
    #        if(compenetration[i]>0):
    #            ref_force[i] = compenetration[i]*stiffness[i]
        
    

    
    ####################################################################### 
    ####################################################################### 
    #                    DRIVE MOTORS
    ####################################################################### 
    ####################################################################### 
    

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



    ####################################################################### 
    ####################################################################### 
    #                    REAL TIME SYNCHRONIZATION
    ####################################################################### 
    ####################################################################### 
    
    time.sleep(0.006)    
    counter = counter+1
    if counter > 20:
        counter = 0
        #print "Status: ", status, " Mode: ", mode
        #print "Pos Ref: ", ref_pos, " Force Ref: ", ref_force
        #print "Pos: ", pos, " Force: ", force, " Pwm: ", pwm_value
        print  "Mode: ", mode, "Pos: ", pos, " Force: ", force, " F Ref: ", ref_force, " Pwm: ", pwm_value
        #print "Status: ", status, " Mode: ", mode, " Force: ", force, " Pwm: ", pwm_value
        #print "Pos: ", pos
        #print "Status: ", status, " Mode: ", mode, "Pos: ", pos, " Joints: ", joint_pos, " Force: ", force
        #print "Mode: ", mode, "Force rectified: ", force_rectified, " Force: ", force, " Filtered Force: ", filtered_force 
    
#ADCcapture.stop()
#ADCcapture.wait()
#ADCcapture.close()