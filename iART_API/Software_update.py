#/usr/bin/env python

#*******************************************************************************
#
# FILE     : Software_update.py
# DATE     : Mon, August 22, 2016
# PROLOGUE : This file implements simple Python script to carry out linux shell
#            scripts from iART_update.sh. The scripts facilitate iART software 
#            update.
# CPU TYPE : ARM Cortec - A53 with BMC2837 core
# PROJECT  : iART
# AUTHOR   : JJ
# VERSION  : ??????
# HISTORY  :
#   JJ1220816 - Start coding:
#   JJ1220816 - 
#   JJ3150616 - 
#   JJ3150616 - 
#   JJ3150616 - 
#   JJ3150616 - 
#   JJ4160616 - 
#   JJ4160616 - 
#   JJ1130616 - 
#   JJ2160816 - 
#   JJ2160816 - 
#
#*******************************************************************************

# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

#from __future__ import division

#import re
#import struct
import os  #JJ1220816 - Need this
#import time
#import sys
#import InHouseAPI
#import Thermostat75C
#import barda
#import smbus     #JJ4180516 - For I2C protocol
#import usb.core  #JJ2190416 - Testing USB
#import usb.util  #JJ2190416 -
#import RPi.GPIO as GPIO  #JJ2260416 - Import GPIO class


# FPGA Register Map
#JJ4050516 REG_FPGA_VERSION = 0x00
#JJ4050516 REG_PCB_VERSION = 0X01

#JJ3150616 - Main menu table
menuItem = ["1) Speaker",
            "2) Temperature",
            "3) Ambient Sensor",
            "4) Battery",
            "5) Power Monitor",
            "6) Motor",
            "7) Home Switch",
            "8) Exit Switch",
            "9) Disposalbe Detection",
            "10) Door Detection",
            "11) Collision Switch",
            "12) Quit"]
maxMenuItem = 12  #JJ3150616 - This number must match the number of items in the menu
menuPrompt = "Key in '1' to '"+str(maxMenuItem)+"' for your selection: "

byteBuffer = bytearray([0,0,0,0,0,0,0,0]) #JJ4190516 - I/O byte buffer
#outBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ4190516 - Output buffer to write to EEPrompt
#inBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])   #JJ4190516 - Input buffer to contain page read from EEPrompt

#tempMSB = 0
#tempLSB = 0


print("Python V3.4.2.")
os.system('bash iART_update.sh')
print("Good bye .....")
#time.sleep(0.25)

"""
if __name__ == "__main__":
    print("Testing Buzzer")
    print("Good bye .....")
    time.sleep(0.5)
"""


#JJ1220816 ----- End of Software_update.py file -----
