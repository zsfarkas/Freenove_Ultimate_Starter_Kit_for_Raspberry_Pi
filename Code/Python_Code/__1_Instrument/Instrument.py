#!/usr/bin/env python3
#############################################################################
# Filename    : ADC.py
# Description : Analog and Digital Conversion, ADC and DAC
# Author      : www.freenove.com
# modification: 2019/12/27
########################################################################
import smbus
import time
import RPi.GPIO as GPIO
import math

address = 0x48  # default address of PCF8591
bus=smbus.SMBus(1)
cmd=0x40        # command, 0100 0000

buzzerPin = 11    # define the buzzerPin
buttonPin = 12    # define the buttonPin


def setup():
    GPIO.setmode(GPIO.BOARD)         # Use PHYSICAL GPIO Numbering
    GPIO.setup(buzzerPin, GPIO.OUT)   # set RGBLED pins to OUTPUT mode
    
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # Set buttonPin to INPUT mode, and pull up to HIGH level, 3.3V
    global p 
    p = GPIO.PWM(buzzerPin, 1) 
    p.start(0)

def alertor(x=200):
    p.start(50)
    # sinVal = math.sin(x / 2 * (math.pi / 180.0))        # calculate the sine value
    toneVal = 440 + x * 3 # Add to the resonant frequency with a Weighted
    p.ChangeFrequency(toneVal)      # Change Frequency of PWM to toneVal
        
def stopAlertor():
    p.stop()

def analogRead(chn): # read ADC value,chn:0,1,2,3
    value = bus.read_byte_data(address,cmd+chn)
    return value
    
def analogWrite(value): # write DAC value
    bus.write_byte_data(address,cmd,value)  
    
def loop():
    while True:
        value = analogRead(0)   # read the ADC value of channel 0
        analogWrite(value)      # write the DAC value to control led
        voltage = value / 255.0 * 3.3  # calculate the voltage value
        print ('ADC Value : %d, Voltage : %.2f'%(value,voltage))
        if GPIO.input(buttonPin)==GPIO.LOW:
            alertor(value)
        else:
            stopAlertor()
        time.sleep(0.01)

def destroy():
    bus.close()
    stopAlertor()
    GPIO.output(buzzerPin, GPIO.LOW)     # Turn off buzzer
    GPIO.cleanup()   

    
if __name__ == '__main__':   # Program entrance
    print ('Program is starting ... ')
    setup()
    try:
        loop()
    except KeyboardInterrupt: # Press ctrl-c to end the program.
        destroy()
        
    
