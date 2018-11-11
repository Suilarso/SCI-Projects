#/usr/bin/env python

#*******************************************************************************
#
# FILE     : Thermostat75C.py
# DATE     : Thur, June 9, 2016
# PROLOGUE : This file implements thermostat class which is the API between a
#            Python script and the BARDA applications.
# CPU TYPE : ARM Cortec - A53 with BMC2837 core
# PROJECT  : iART
# AUTHOR   : JJ
# VERSION  : ??????
# HISTORY  :
#   JJ3250516 - Start coding:
#   JJ3250516 - Create Thermostat class definition
#   JJ3250516 - Add method to set / get configuration register
#   JJ3250516 - Add method to start / stop temperature conversion
#   JJ4260516 - Add method to get temperature; getTemperature()
#   JJ2310516 - Modify codes to used TI TMP75 device instead of original DS1631
#   JJ4090616 - Rename file to ...75C and modify codes for TI TMP75C
#   JJ1130616 - Test run the code on TI evaluation board
#
#*******************************************************************************

# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

#from __future__ import division

#import re
#import struct
import time
import sys
import smbus     #JJ4180516 - For I2C protocol
#import usb.core  #JJ2190416 - Testing USB
#import usb.util  #JJ2190416 -
#import RPi.GPIO as GPIO  #JJ2260416 - Import GPIO class


# GPIO pin usage on the Rev 1.0 PCB.  Note that these are PHYSICAL pin numbers on the header and not Broadcom pins
# FPGA configuration pins

# I2C interface
I2C_SDA = 3
I2C_SCL = 5
# SPI interface 

# FPGA Register Map
#JJ4050516 REG_FPGA_VERSION = 0x00
#JJ4050516 REG_PCB_VERSION = 0X01

# SPI Bus literals


#FPGA_CLK_RATE = 125000000.0

# Motor parameters

# Masks for manipulating the motor control register

#JJ4190516 - Create a class by the name of Thermostat
#JJ4190516 - Methods will be as follow: Set register address, Read device register, Write device register

#JJ2310516 - Both DS1631 and TMP75 use the same slave address structure
THERMO_ADDR = 0x48   #JJ3250516 - DS1631 Thermometer & Thermostat Device (A0, A1, A2 connect to ground)
#RTC_ADDR_WR = 0xD0 #JJ4050516 - 11010000
#RTC_ADDR_RD = 0xD1 #JJ4050516 - 11010001

I2CBus = smbus.SMBus(1)

#JJ3010616 - Below is TMP75 Register (Command) Set
TEMPERATURE_REG = 0x00  #JJ3010616 - Command to read converted temperature
CONFIG_REG = 0x01  #JJ3010616 - Command to configure TMP75 operation
T_LOW_REG = 0x02  #JJ3010616 - Command to set threshold low register
T_HIGH_REG = 0x03  #JJ3010616 - Command to set threshold high register
ONE_SHOT_REG = 0x04  #JJ4090616 - Write anything to activate one-shot conversion

#JJ3010616 - Below is configuration register bit format
SD_BIT = 0x0100
TM_BIT = 0x0200
POL_BIT = 0x0400
F0_BIT = 0x0800
F1_BIT = 0x1000
OS_BIT = 0x2000


DEFAULT_CONFIG = F1_BIT | F0_BIT  #JJ3010616 - equiv 0x1800
UPPER_THRESHOLD = 0x4B00  #JJ4090616 - Set to 75; subject to change
LOWER_THRESHOLD = 0xE700  #JJ4090616 - Set to -25; subject to change

byteBuffer = bytearray([0,0,0,0,0,0,0,0]) #JJ4190516 - I/O byte buffer
outBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ4190516 - Output buffer to write to EEPrompt
#inBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])   #JJ4190516 - Input buffer to contain page read from EEPrompt

tempMSB = 0
tempLSB = 0

#JJ3250516 - Thermostat Class definition
class Thermostat(object):
    """ JJ3250516 This class provide API for thermometer device DS1631 to configure its, """
    """ registers."""

    def __init__(self):
        global UPPER_THRESHOLD
        #I2CBus = smbus.SMBus(1)
        print("Creating Thmermostat class.")
        try:
            self.setConfiguration()  #JJ4090616 - Official code; by-pass for debugging purpose
        except:
            raise Exception("Temperature Error")

    def readFromThermometerDevice(self, command, size):
        """ JJ3250516 - This method read a byte / word value from device in response to a command """
        """ JJ3250516 - This method needs to be tested with actual device when it becomes available """
        """ JJ3250516 - This method may not be actually used """
        if size == 1:
            retValue = I2CBus.read_byte_data(THERMO_ADDR, command)  #JJ3250516 - OFFICIAL CODE
        else:
            retValue = I2CBus.read_word_data(THERMO_ADDR, command)  #JJ3250516 - OFFICIAL CODE
        return retValue

    def writeToThermometerDevice(self, command):
        """ JJ3250516 - This method write data to ambient sensor register """
        global byteBuffer
        I2CBus.write_i2c_block_data(THERMO_ADDR, command, byteBuffer)

    def startMeasure(self):
        """ JJ3250516 - This method start temperature conversion """
        """ JJ3250516 - According to spec pg 11, it only need dev address, and command """
        """ JJ3250516 - However, raspberry I2C library write device address, command, and """
        """ JJ3250516 - and additional byte. It does not have a library that just wirte """
        """ JJ3250516 - device address and command. NEED TO THROUGHLY TEST THIS METHOD """
        #I2CBus.write_byte_data(THERMO_ADDR, START_CONVERT_T, START_CONVERT_T)
        pass

    def stopMeasure(self):
        """ JJ3250516 - This method stop temperature conversion """
        """ JJ3250516 - According to spec pg 11, it only need dev address, and command """
        """ JJ3250516 - However, raspberry I2C library write device address, command, and """
        """ JJ3250516 - and additional byte. It does not have a library that just wirte """
        """ JJ3250516 - device address and command. NEED TO THROUGHLY TEST THIS METHOD """
        #I2CBus.write_byte_data(THERMO_ADDR, STOP_CONVERT_T, STOP_CONVERT_T)
        pass

    def getConfiguration(self):
        """ JJ3250516 - This method return the value from the configuration register """
        """ JJ3250516 - ??? """
        #global tempMSB
        #global tempLSB
        global byteBuffer
        configReg = 0x0000
        try:
            byteBuffer = I2CBus.read_i2c_block_data(THERMO_ADDR, CONFIG_REG, 2)
            #configReg = self.I2CBus.read_byte_data(THERMO_ADDR, CONFIG_REG)
            configReg = (byteBuffer[0] << 8) | byteBuffer[1]
        except:
            print("\ngetConfiguration end with error .....")
        return configReg

    def setConfiguration(self):
        """ JJ3250516 - This method set configuration register to a specified setting. """
        """ JJ3090616 - Here we set T_High register to 75, T_Low register to -25, and """
        """ JJ3250516 - configuration register to 0x1800 (FQ1, FQ0 both to high). """
        global UPPER_THRESHOLD
        global LOWER_THRESHOLD
        global outBuffer
        try:
            #JJ4090616 - Write to T_HIGH_REG (0x03)
            outBuffer[0] = UPPER_THRESHOLD >> 8  #JJ4090616 - Retrieve MSB
            outBuffer[1] = UPPER_THRESHOLD & 0x00FF  #JJ4090616 - Retrieve LSB
            outList = list(outBuffer[0:2])
            I2CBus.write_i2c_block_data(THERMO_ADDR, T_HIGH_REG, outList)  #JJ3010616 - Official
            #JJ4090616 - Write to T_LOW_REG (0x02)
            outBuffer[0] = LOWER_THRESHOLD >> 8  #JJ4090616 - Retrieve MSG
            outBuffer[1] = LOWER_THRESHOLD & 0x00FF  #JJ4090616 - Retrieve LSB
            outList = list(outBuffer[0:2])
            I2CBus.write_i2c_block_data(THERMO_ADDR, T_LOW_REG, outList)  #JJ3010616 - Official code
            #JJ4090616 - Write to CONFIG_REG (0x01)
            outBuffer[0] = DEFAULT_CONFIG >> 8  #JJ3090616 - Retrieve MSG
            outBuffer[1] = DEFAULT_CONFIG & 0x00FF  #JJ3090616 - Retrieve LSB
            outList = list(outBuffer[0:2])
            I2CBus.write_i2c_block_data(THERMO_ADDR, CONFIG_REG, outList)  #JJ3010616 - Official code
        except:
            print("\nsetConfiguration end with error .....")
            raise Exception("Temperature Error")

    def getTemperature(self):
        """ JJ3250516 - This method read raw 2-complements data from temperature register """
        """ JJ3250516 - It converts the 2-complement into float value and return it to calling """
        """ JJ3250516 - function """
        #global tempMSB
        #global tempLSB
        global byteBuffer
        byteBuffer = bytearray([0,0,0,0,0,0,0,0])
        #for i in range (0, 5):
        #    byteBuffer[i] = 0
        resolution = 0.0625
        temperature = 0.0
        constOffset = 0  #JJ5211016 - Pending for a constant value
        try:
            byteBuffer = I2CBus.read_i2c_block_data(THERMO_ADDR, TEMPERATURE_REG, 2)  #JJ4090616 - Official
            rawData = (byteBuffer[0] << 8) | byteBuffer[1]  #JJ4090616 - Official code
            #rawData = 0x7FF0  #JJ4260516 - 127.9375
            #rawData = 0x0040  #JJ4260516 - 0.25
            #rawData = 0xFFC0  #JJ4260516 - -0.25
            #rawData = 0xC900  #JJ4260516 - -55
            #rawData = 0x0000  #JJ4260516 - 0
            #JJ4260516 - If rawData is negative, reverse all bits as it is 2-complement value
            if rawData & 0x8000:
                rawData = ~rawData
                rawData = rawData & 0xFFFF  #JJ4090616 - We only interested in 2 bytes
                #JJ4260516 - Since rawData is 12 bits resolution, we shift right 4 bits
                rawData = (rawData >> 4) + 1
                negValue = -1.0
            #JJ4260516 - rawData is positive; 12 bits resolution, shift right 4 bits
            else :
                rawData = rawData & 0xFFFF  #JJ4090616 - Ensure we use only 2 bytes
                rawData = rawData >> 4
                negValue = 1.0

            temperature = (rawData * resolution * negValue) - constOffset  #JJ1171016 - Subtract tolerance
        except:
            print("\ngetTemperature end with error .....")

        #print("\nTemperature is: ", temperature)
        #return int(temperature)  #JJ2140616 - JJNOTE: Other usage may need float value.
        return (temperature)  #JJ1171016 - Return float value to caller.


if __name__ == "__main__":
    testObj = Thermostat()
    while True:
        temp = testObj.getTemperature()
        print("\nRounded temperature: ", temp)
        time.sleep(1)
    time.sleep(0.5)

#bus = smbus.SMBus(1)

#JJ EEPromptPage = (501 << 6) + 0x00
#JJ outBuffer.append(EEPromptPage >> 8)  #JJ4140416 - MSB of EEPrompt internal address
#JJ outBuffer.append(EEPromptPage & 0x00FF)  #JJ4140416 - LSB of EEPrompt internal address
#bus.write_i2c_block_data(DEVICE_ADDRESS, outBuffer[0], outBuffer[1:])
#JJ4140416 - Next we need to read in byte by byte the next 64 bytes of a page
#for i in range (1, 64):
#    inBuffer.append(bus.read_byte(DEVICE_ADDRESS))


#JJ1250416 - Below code added from PyUSB/Mailing Lists


#time.sleep(1)



#JJ3250516 ----- End of Thermostat.py file -----
