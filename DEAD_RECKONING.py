import RPi.GPIO as GPIO
import time
from AlphaBot import AlphaBot
import math as m

Ab = AlphaBot();
Ab.stop();

cntl = 7;
cntr = 8;

EncR = 0.0;
EncL = 0.0;

def updateEncoderL(channel):
    global EncL;
    EncL += 1;
    #print ('valEncL = %d' %EncL)
    
def updateEncoderR(channel):
    global EncR;
    EncR += 1;
    #print ('valEncR = %d' %EncR)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False);
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)

if __name__ == "__main__":
    '''
    1 turn of a wheel will result in 40 counts of encoder.
    '''        
    R0 = EncR;
    nt = 1;
    
    t=0.30;
    
    """t = time taken for the bot to travel right turn for 90 degrees at speed 30
    """
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,40)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    X0=0.0;
    Y0=0.0;
    theta0=0.0;
    #motor speed for turning the bot right hand side with respect to spatial frame.
    a=30;
    b=30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta1=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta1=theta1;
    else:
        theta1=-theta1;
    print ('theta1 = %d' %theta1)    
    w=theta1*m.pi/180.0;   
    X1=X0 + (d*m.cos(w));
    Y1=Y0 + d*m.sin(w);
    print ('X1 = %d' %X1)
    print ('Y1 = %d' %Y1)

    '''
    SECOND TURN
    '''
    t=0.33;
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,40)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    a=-30;
    b=-30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta2=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta2=theta2;
    else:
        theta2=-theta2;
    theta2=theta1+theta2;
    print ('theta2 = %d' %theta2)
    
    w=theta2*m.pi/180.0;   
    X2=X1 + (d*m.cos(w));
    Y2=Y1 + d*m.sin(w);
    print ('X2 = %d' %X2)
    print ('Y2 = %d' %Y2)
    
    '''
    THIRD POINT
    '''
    t=0.37;   
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,38)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    a=-30;
    b=-30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta3=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta3=theta3;
    else:
        theta3=-theta3;
    theta3=theta2+theta3;
    print ('theta3 = %d' %theta3)
    
    w=-(theta3)*m.pi/180.0;   
    X3=X2 + (d*m.cos(w));
    Y3=Y2 + d*m.sin(w);
    print ('X3 = %d' %X3)
    print ('Y3 = %d' %Y3)
    
    
    '''
    FOURTH POINT
    '''
    
    t=0.34;   
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,37)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    a=30;
    b=30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta4=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta4=theta4;
    else:
        theta4=-theta4;
    theta4=theta3+theta4;
    print ('theta4 = %d' %theta4)
    
    w=theta4*m.pi/180.0;   
    X4=X3 -(d*m.cos(w));
    Y4=Y3 + d*m.sin(w);
    print ('X4 = %d' %X4)
    print ('Y4 = %d' %Y4)
    
    '''
    FIFTH POINT
    '''
    t=0.30;   
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,40)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    a=30;
    b=30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta5=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta5=theta5;
    else:
        theta5=-theta5;
    theta5=theta4+theta5;
    print ('theta5 = %d' %theta5)
    
    w=theta5*m.pi/180.0;   
    X5=X4 + (d*m.cos(w));
    Y5=Y4 + d*m.sin(w);
    print ('X5 = %d' %X5)
    print ('Y5 = %d' %Y5)
    
    '''
    Sixth point
    '''
    t=0.30;   
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,40)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    a=30;
    b=30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta6=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta6=theta6;
    else:
        theta6=-theta6;
    theta6=theta5+theta6;
    print ('theta6 = %d' %theta6)
    
    w=theta6*m.pi/180.0;   
    X6=X5 -(d*m.cos(w));
    Y6=Y5 + d*m.sin(w);
    print ('X6 = %d' %X6)
    print ('Y6 = %d' %Y6)
    
    '''
    SEVENTH POINT
    '''
    t=0.30;   
    t1=1;
    # Make nt turns
    '''
    Move forward for the first stop.
    '''
    EncR=0;
    EncL=0;
    
    while EncR < 30:
        Ab.setMotor(-40,40)
    Ab.stop()
    #print ('EncR = %d' %EncR)
    d=EncR * 0.8;
    time.sleep(1)
    #initilize coordinates
    EncR=0;
    EncL=0;
    a=30;
    b=30;
    Ab.setMotor(a,b)
    time.sleep(t)
    print ('EncR=%d' %EncR)
    Enc=max(EncR,EncL)
    theta7=(Enc*8);
    Ab.stop()
    time.sleep(2)
    # for 180 degree turn around the axis of robot it takes 0.74 microseconds
    #as per design it must turn ninety and it takes 35 encoder counts for 180 degrees
    #then 90 degree angle is given  by the relation (one count= 5.1428 degrees)
  
    if a>0 and b>0:
        theta7=theta7;
    else:
        theta7=-theta7;
    theta7=theta6+theta7;
    print ('theta7 = %d' %theta7)
    
    w=theta7*m.pi/180.0;   
    X7=X6 + (d*m.cos(w));
    Y7=Y6 + d*m.sin(w);
    print ('X7 = %d' %X7)
    print ('Y7 = %d' %Y7)