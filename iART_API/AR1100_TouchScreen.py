#/usr/bin/env python

#JJ3270416 - 
#*******************************************************************************
#
# FILE     : AR1100_TouchScreen.py
# DATE     : Wed, April 27, 2016
# PROLOGUE : This file implements the API between for LCD screen calibration
#
# CPU TYPE : ARM Cortex - A53 with BMC2837 core
# PROJECT  : iART
# AUTHOR   : JJ
# VERSION  : ??????
# HISTORY  :
#   JJ3270416 - Start coding touch screen:
#   JJ3270416 - Create AR1100_TouchScreen class definition
#   JJ3290616 - Resume coding actiity
#   JJ5110816 - Calibration issues seem to have been solved
#
#*******************************************************************************

#
#JJ3270416 - This file implements the AR1100 Touch Screen class which is the API between
#JJ3270416 - a Python script and AR1100 touch screen to facilitate calibration process.
# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

#from __future__ import division

#import re
#import struct
import time
import sys
#import smbus     #JJ4140416 - For I2C protocol
import usb.core  #JJ2190416 - Testing USB
import usb.util  #JJ2190416 -
#import RPi.GPIO as GPIO  #JJ2260416 - Import GPIO class
#import subprocess  #JJ3270416 - For testing buzzer



# GPIO pin usage on the Rev 1.0 PCB.  Note that these are PHYSICAL pin numbers on the header and not Broadcom pins
# FPGA configuration pins

# I2C interface
#JJ3270416 I2C_SDA = 3
##JJ3270416 I2C_SCL = 5

# SPI interface 
#JJ3270416 SPI_MOSI = 19
#JJ3270416 SPI_MISO = 21
#JJ3270416 SPI_SCLK = 23
#JJ3270416 SPI_SS_N = 24

# SPI Bus literals
#JJ3270416 SPI_READ = 0x80
#JJ3270416 SPI_WRITE = 0x00

#FPGA_CLK_RATE = 125000000.0  #JJ3270416 - 125MHz ???

# Motor parameters

# Masks for manipulating the motor control register

#JJ3270416 - Global variables
commandBuf = []  #JJ4140416 - Buffer to contain commands for AR1100
responseBuf = []   #JJ4140416 - Buffer to contain response from AR1100
#bytearray()

AR1100_VID = 0x04D8
HID_MOUSE_PID = 0x0C02  #JJ4280416 - HID-MOUSE
HID_GENERIC_PID = 0x0C01  #JJ4280416 - HID_GENERIC

#JJ3290616 - Command set for AR1100; refer to pg 22 to pg 24
TOUCH_ENABLE_CMD = 0x12
TOUCH_DISABLE_CMD = 0x13
CALIBRATE_CMD = 0x14
REG_READ_CMD = 0x20
REG_WRITE_CMD = 0x21
#EE_READ_CMD = 0x28
#EE_WRITE_CMD = 0x29
USB_MODE_GENERIC_CMD = 0x70
USB_MODE_MOUSE_CMD = 0x71
#USB_MODE_DIGITIZER = 0x72  #JJ3290616 - Very unlikely we use it, so remark

#JJ3290616 - Status for response packet; refer to pg 21 of AR1100 datasheet
OK = 0x00
UNRECOGNIZED = 0x01
TIMEOUT = 0x04
EEPARAMS_ERR = 0x05
CAL_CANCEL = 0xFC


#JJ1250716 - LCDCalibration Class definition -----------------------------------
class LCDCalibration(object):
    """ JJ1250716 - This class provide API for AR1100 LCD calibration """

    def __init__(self):
        self.touchDev = 0
        self.readEP = 0
        self.writeEP = 0
        #pass

    #JJ5050816 - This method change device operation from fromMode to toMode
    #JJ1171016 - Not used at the moment.
    def changeDeviceMode(self, fromMode, toMode):
        returnFlag = 0
        self.touchDev = usb.core.find(idVendor=AR1100_VID, idProduct=fromMode)#HID_MOUSE_PID)
        if self.touchDev:
            interface = 0
            endpoint = self.touchDev[0][(0,0)][0]
            self.readEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for read - 0x81
            endpoint = self.touchDev[0][(0,0)][1]
            self.writeEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for write - 0x01
            if self.touchDev.is_kernel_driver_active(interface) == True:
                self.touchDev.detach_kernel_driver(interface)
                usb.util.claim_interface(self.touchDev, interface)
            try:
                #JJ5050816 - Change to comand 0x70 to change to Generic Mode
                if fromMode == HID_MOUSE_PID:
                    #                             bReq, wVal,   wIndex, data
                    ret = self.touchDev.ctrl_transfer(0x21, 0x09, 0x0001, 0x0000, [0x55, 0x01, 0x70])
                else:
                    #                             bReq, wVal,   wIndex, data
                    ret = self.touchDev.ctrl_transfer(0x21, 0x09, 0x0001, 0x0000, [0x55, 0x01, 0x71])
                if (ret == 3):  #JJ5050816 - Seccessfully turned into GENERIC
                    #print ("!")
                    self.touchev = usb.core.find(idVendor=AR1100_VID, idProduct=toMode)#HID_GENERIC_PID)
                    if self.touchev:
                        interface = 0
                        endpoint = self.touchDev[0][(0,0)][0]
                        self.readEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for read - 0x81
                        endpoint = self.touchDev[0][(0,0)][1]
                        self.writeEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for write - 0x01
                        while True:
                            try:
                                #self.touchDev.reset()
                                if self.touchDev.is_kernel_driver_active(interface) == True:
                                    self.touchDev.detach_kernel_driver(interface)
                                    usb.util.claim_interface(self.touchDev, interface)
                                time.sleep(1)
                                self.touchDev.set_configuration()
                            except:
                                returnFlag = -1

                    else:
                        print("Failed to change to GENERIC MODE")
                        returnFlag = -1
            except:
                #JJ5050816 - Failed to get data for this request
                print ("Failed to turn into HID")
                returnFlag = -1
        else:
            returnFlag = -1

    #JJ1171016 - This method is not used.
    def getDeviceConnection(self):
        returnFlag = 0
        self.touchDev = usb.core.find(idVendor=AR1100_VID, idProduct=0x0C02)
        if self.touchDev is not None:
            endpoint = self.touchDev[0][(0,0)][0]
            self.readEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for read - 0x81
            endpoint = self.touchDev[0][(0,0)][1]
            self.writeEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for write - 0x01
            self.touchDev.reset()
            if self.touchDev.is_kernel_driver_active(0) == True:
                self.touchDev.detach_kernel_driver(0)
                #usb.util.claim_interface(self.touchDev, 0)
            #time.sleep(1)
            self.touchDev.set_configuration()  #JJ3270416 - Set configuration
        else:
            returnFlag = -1

    #JJ3270416 - This method gets device, read endpoint, and write endpoint
    #JJ2260716 - Return 0 if no error, -1 if there is error
    #JJ4040816 - 0x0C01 = 3073 - Generic
    #JJ4040816 - 0x0C02 = 3074 - Mouse
    def getDeviceConfiguration(self, productID=0x0C02):
        returnFlag = 0
        #self.touchDev = usb.core.find(idVendor=AR1100_VID, idProduct=HID_MOUSE_PID)
        self.touchDev = usb.core.find(idVendor=AR1100_VID, idProduct=productID)
        #JJ1250416 - Below code added from PyUSB/Mailing Lists
        if self.touchDev is not None:
            self.touchDev.reset()
            if self.touchDev.is_kernel_driver_active(0) == True:
                self.touchDev.detach_kernel_driver(0)
            #time.sleep(1)
            self.touchDev.set_configuration()  #JJ3270416 - Set configuration
            endpoint = self.touchDev[0][(0,0)][0]
            self.readEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for read - 0x81
            endpoint = self.touchDev[0][(0,0)][1]
            self.writeEP = endpoint.bEndpointAddress  #JJ3270416 - Will get endpoint for write - 0x01
            #cfg = self.touchDev.get_active_configuration()  #JJ3270416 - Get endpoint instance
            #intf = cfg[(0,0)]  #JJ3270416 - Get interface
            #ep = usb.util.find_descriptor(intf, custom_match = lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)  #JJ3270416 - Get descriptor
            #assert ep is not None
            #self.touchDev.attach_kernel_driver(0)
        else:
            #Return to caller function with error message in USB
            returnFlag = -1

        return (returnFlag)
    #JJ1110716 - End of getDeviceConfiguration()

    #JJ4300616 - This method issue command to change USB mode to the one specified in newMode
    #JJ1171016 - Not used at the moment. KIV
    def changeUSBMode(self, newMode=USB_MODE_MOUSE_CMD):
        global commandBuf
        commandBuf.clear()
        commandBuf.insert(0, 0x55)  #JJ4300616 - SYNC
        commandBuf.insert(1, 1)  #JJ4300616 - SIZE  #JJ2260416 - Hard coded here
        #default = USB_MODE_MOUSE_CMD, USB_MODE_GENERIC_CMD
        commandBuf.insert(2, newMode)  #JJ4300616 - Read command = newMode
        self.touchDev.write(self.writeEP, commandBuf, 1000)
        #time.sleep(0.5)
        print("USB Command: "+str(newMode))
    #JJ1110716 - End of changeUSBMode()

    #JJ1110716 - This method set enable or disable command to USB device
    def setDeviceTouchMode(self, newMode=TOUCH_ENABLE_CMD):
        global commandBuf
        global responseBuf
        #responseBuf.clear()
        #respBuf = bytearray([0,0,0,0])
        commandBuf.clear()
        commandBuf.insert(0, 0x55)  #JJ4300616 - SYNC
        commandBuf.insert(1, 1)  #JJ4300616 - SIZE  
        commandBuf.insert(2, newMode)  #JJ1110716 - newMode
        self.touchDev.write(self.writeEP, commandBuf, 2000)
        time.sleep(0.5)  #(3)
        responseBuf = self.touchDev.read(self.readEP, 4, 2000)
        print("Device touch mode: ", responseBuf)
        #responseBuf = self.touchDev.read(self.readEP, respBuf, 1000)
        pass

    #JJ4210716 - This method activate calibration process
    def setCalibrateCommand(self, newType=0x02):
        global commandBuf
        global responseBuf
        global OK
        numByte = 4  #JJ4280716 - Used if verbose is off
        #numByte = 9  #JJ4280716 - Used if verbose is on
        retFlag = 0
        commandBuf.clear()
        commandBuf.insert(0, 0x55)  #JJ4210716 - SYNC
        commandBuf.insert(1, 2)  #JJ4210716 - SIZE  #JJ2260416 - Hard coded here
        commandBuf.insert(2, 0x14)  #JJ4210716 - COMMAND, calibrate
        commandBuf.insert(3, newType)  #JJ4210716 - Calibration type 0x02 is 9 points
        self.touchDev.write(self.writeEP, commandBuf, 2000)
        #JJ5050816 - wValue = 0x0001 - bConfigurationValue = 1
        #self.touchDev.ctrl_transfer(0x21, 0x09, 0x0001, 0x0000, [0x55, 0x02, 0x14, 0x02])
        time.sleep(0.5) #(1)  #JJ5120816 - Reduce to half a second
        responseBuf = self.touchDev.read(self.readEP, numByte, 2000)
        if numByte == 9:
            if responseBuf[4] == 0xFE:  #JJ4210716 - 0xFE indicates enter calibration mode
                retFlag = 1
        else:
            if responseBuf[2] == OK:
                retFlag = 1
        return retFlag

    #JJ5220716 - This method get device's reponse for each target being touched
    #JJ5120816 - Return: 0 - if time out; 1 - user responded, proceed to subsequent point 
    def getTouchResponse(self, verbose=0x00):
        global responseBuf
        retValue = 0
        timeOut = 6  #JJ5120816 - (6 * 5000 = 30000) 30 seconds allowed for non-response. After which considered incomplete.
        numByte = 4
        if verbose == 0x01:
            numByte = 9
        #responseBuf.clear()
        while timeOut > 0:
            try:
                #JJ5110816 - Device response to user touch
                #responseBuf = self.touchDev.read(self.readEP, 15, 5000)
                #startAt = time.clock()  #JJ1071116 - Starting time
                startAt = time.time()  #JJ1071116 - Starting time
                responseBuf = self.touchDev.read(self.readEP, numByte, 5000)
                #finishAt = time.clock()  #JJ1071116 - Ending time
                finishAt = time.time()  #JJ1071116 - Ending time
                timeTaken = finishAt - startAt
                #print("time take: ", timeTaken)
                #JJ5120816 - Below if sttmt may not be needed if verbose pin in TouchOptions is not set
                #if verbose == 0x01:
                #    point = int(responseBuf[4]) + 1  #JJ5220716 - Point number being touched
                #    xCoord = int ((responseBuf[5] << 8) | responseBuf[6])
                #    yCoord = int ((responseBuf[7] << 8) | responseBuf[8])
                #    print("Point #: "+str(point))
                #    print("X Y: "+str(xCoord)+"  "+str(yCoord))
                #if responseBuf[2] == 0x00:  #JJ2081116 - ORG
                if timeTaken > 0.45 and responseBuf[2] == 0x00:
                    print(responseBuf)
                    retValue = 1
                    break;
                elif timeTaken <= 0.45:
                    retValue = 2
                    break;
            except:
                time.sleep(0.5)
                timeOut = timeOut - 1

        #JJ5120816 - Time out, need to cancel calibration command, by issuing any command
        #JJ5120816 - in our case, we issue touch enable command
        if timeOut == 0 or retValue == 2:
            self.setDeviceTouchMode()

        return (retValue)

    #JJ1110716 - This method read register's contents
    def readFromRegister(self, regAddr, readSize):
        global commandBuf
        global responseBuf
        #responseBuf.clear()
        commandBuf.clear()
        commandBuf.insert(0, 0x55)  #JJ1110716 - SYNC
        commandBuf.insert(1, 4)  #JJ1110716 - SIZE  #JJ1110716 - Hard coded here
        commandBuf.insert(2, REG_READ_CMD)  #JJ1110716 - Read command = 0x20
        commandBuf.insert(3, 0x00)  #JJ1110716 - Register Addr (MSB)
        commandBuf.insert(4, regAddr)  #JJ1110716 - Register Addr (LSB), LSB of the register to read from
        commandBuf.insert(5, readSize)  #JJ1110716 - Number of bytes to read from register
        self.touchDev.write(self.writeEP, commandBuf, 1000)
        time.sleep(0.75)
        #responseBuf[4] = 0x00
        responseBuf = self.touchDev.read(self.readEP, 9, 1000)
    #JJ1110716 - End of readFromRegister()

    #JJ2120716 - This method writes data to register
    def writeToRegister(self, regAddr, data):
        global commandBuf
        global responseBuf
        commandBuf.clear()
        commandBuf.insert(0, 0x55)  #JJ2120716 - SYNC
        commandBuf.insert(1, 4)  #JJ2120716 - SIZE  #JJ2120716 - Hard coded here
        commandBuf.insert(2, 0x21)  #JJ2120716 - COMMAND
        commandBuf.insert(3, 0x00)  #JJ2120716 - Register Addr (MSB)
        commandBuf.insert(4, regAddr)  #JJ2120716 - Register Addr (LSB), TouchOption
        commandBuf.insert(5, data)  #JJ2120716 - 
        self.touchDev.write(self.writeEP, commandBuf, 1000)
        time.sleep(2.0)
        responseBuf = self.touchDev.read(self.readEP, 9, 1000)
    #JJ2120716 - End of writeToRegister()

    #JJ2120716 - This method read data from EEPROM at the specified location
    def readFromEEPROM(self, EEAddr, readSize):
        global commandBuf
        global responseBuf
        commandBuf.clear()
        commandBuf.insert(0, 0x55)  #JJ2120716 - SYNC
        commandBuf.insert(1, 4)  #JJ2120716 - SIZE  #JJ2110716 - Hard coded here
        commandBuf.insert(2, EE_READ_CMD)  #JJ2120716 - Read command = 0x28
        commandBuf.insert(3, 0x00)  #JJ2120716 - EEPROM Addr (MSB)
        commandBuf.insert(4, EEAddr)  #JJ2120716 - EEPROM Addr (LSB), LSB of the eeprom to read from
        commandBuf.insert(5, readSize)  #JJ2120716 - Number of bytes to read from eeprom
        self.touchDev.write(self.writeEP, commandBuf, 1000)
        time.sleep(0.75)
        responseBuf = self.touchDev.read(self.readEP, 9, 1000)
    #JJ2120716 - End of readFromEEPROM()

    #JJ2260716 - This method get device for USB mouse mode
    def resolveDeviceProductID(self, product_id):
        #JJ2260716 - Here product_id represent the product id of the intended device;
        #JJ2260716 - Sometime the device could be in the other mode from the intended mode
        print("Intended mode: "+str(product_id))
        retValue = 0
        if product_id == HID_GENERIC_PID:
            otherMode = HID_MOUSE_PID
            USBModeCmd = USB_MODE_GENERIC_CMD
        else:
            otherMode = HID_GENERIC_PID
            USBModeCmd = USB_MODE_MOUSE_CMD

        if self.getDeviceConfiguration(product_id) == -1:  #JJ2260716 - Can't get the device
            print("Fail to go to: "+str(product_id))
            iteration = 5  #JJ2260716 - Try to get other device for 3 times
            while iteration > 0:
                time.sleep(0.5) #(1)
                if self.getDeviceConfiguration(otherMode) == 0:
                    self.touchDev.reset()
                    print("Change to mode: "+str(otherMode))
                    #time.sleep(0.5) #(1)
                    self.changeUSBMode(USBModeCmd)
                    break
                iteration -= 1

            if iteration == 0:  #JJ2260716 - Still can't find after three trial; give up
                print("Give Up.")
                retValue = -1
            else:
                #JJ2260716 - Now try to go to the intended mode again
                iteration = 5
                while iteration > 0:
                    time.sleep(0.5) #(0.3)
                    if self.getDeviceConfiguration(product_id) == 0:
                        self.touchDev.reset()
                        self.changeUSBMode()
                        print("Finally Get to mode: "+str(product_id))
                        break
                    iteration -= 1

                if iteration == 0:  #JJ2260716 - Still can't find intended device; give up
                    print("Total failure")
                    retValue = -1

        return (retValue)
    #JJ2260716 - End of get device USB mouse mode

    #JJ1110716 - Main body for the calibration flow
    #JJ5220716 - This method return 0 if no error, 1 if there are errors
    def activateCalibration(self):
        retStatus = 0
        try:
            #self.changeDeviceMode(HID_MOUSE_PID, HID_GENERIC_PID)
            #self.changeDeviceMode(HID_GENERIC_PID, HID_MOUSE_PID)
            #JJ1110716 - 1) Get device configuration for HID_MOUSE product ID
            #JJ1110716 - 2) Change USB mode to HID_GENERIC
            #JJ1110716 - 3) Get device configuration for HID_GENERIC product ID
            print("Phase 1: get USB mouse device")
            if self.getDeviceConfiguration() == 0:  #JJ1110716 - Get end pointer for USB mouse mode
                skip_Generic = 1
                #time.sleep(0.5)
                #self.changeUSBMode(USB_MODE_GENERIC_CMD)
                #time.sleep(0.5)
                print("Phase 2: get USB generic device")
                #if self.getDeviceConfiguration(HID_GENERIC_PID) == 0:  #JJ1110716 - Get end pointer for USB generic mode
                if skip_Generic == 1:
                    #time.sleep(0.5)
                    #JJ5120816 - Disable USB touch. Monitor if this command will give problem.
                    #JJ5120816 - If it does, remark it as well as the counterpart in exitCalibrationMode()
                    self.setDeviceTouchMode(TOUCH_DISABLE_CMD)
                    #JJ4210716 - Issue calibrate command
                    time.sleep(0.5)
                    print("Phase 3: issue calibrate command")
                    if self.setCalibrateCommand() == 0:
                        retStatus = 1
                        print("Set calibrate command error")
                    #JJ4210716 - Here onward set point 1 to point 9
                else:  #JJ1171016 - This else part is used only if GENERIC mode is used.
                    retStatus = 1
                    #JJ1110716 - Change back to HID_MOUSE mode, if we can't get to HID_GENERIC mode
                    if self.getDeviceConfiguration() == 0:
                        time.sleep(0.3)
                        self.changeUSBMode()
                        print("Fail to get to GENERIC mode; Successfully get back to MOUSE mode")
                    print("Fail phase 2")
            else:  #JJ2120716 - Since we can't get end pointer for HID_MOUSE, try to get HID_GENERIC
                retStatus = 1
                print("Fail phase 1")
        except:
            retStatus = 1
            print("activateCalibration end with error...")

        return (retStatus)

    def exitCalibrationMode(self):
        skip_Generic = 1
        #if getDeviceConfiguration(HID_GENERIC_PID) == 0:
        if skip_Generic == 1:
            #JJ5120816 - Enable USB touch. If this gives problem, remark it as well as the counterpart in activateCalibration()
            self.setDeviceTouchMode()
            usb.util.release_interface(self.touchDev, 0)
            self.touchDev.attach_kernel_driver(0)
        else:
            print("End with error")

#getDeviceConfiguration(vendorID=0x04D8, productID=0x0C02)
#HID_GENERIC_PID
#USB_MODE_MOUSE_CMD

#JJ1250716 - LCDCalibration Class END ------------------------------------------

"""
#JJ3290616 - When reach here, USB communication channel must had been setup
def dummyTest():
    global responseBuf
    #calibrate = LCDCalibration()
    #calibrate.activateCalibration()
    #calibrate.getDeviceConfiguration()
    #calibrate.touchDev.attach_kernel_driver(0)
    while True:
        print("\n\n\n\n\n\n")
        print('Touch any point on the LCD')
        time.sleep(3)
        #JJ3290616 - First read
        responseBuf = calibrate.touchDev.read(calibrate.readEP, 15, 5000)
        mode = int(responseBuf[0])
        xCoord = int (((responseBuf[2] & 0x0F) << 8) | responseBuf[1])
        #xCoord = xCoord >> 2
        yCoord = int (((responseBuf[4] & 0x0F) << 8) | responseBuf[3])
        #yCoord = yCoord >> 2
        print(responseBuf)
        print("Mode: "+str(mode))
        print("X Y: "+str(xCoord)+"  "+str(yCoord))

        #JJ3290616 - Second read
        responseBuf = calibrate.touchDev.read(calibrate.readEP, 15, 5000)
        mode = int(responseBuf[0])
        xCoord = int (((responseBuf[2] & 0x0F) << 8) | responseBuf[1])
        #xCoord = xCoord >> 2
        yCoord = int (((responseBuf[4] & 0x0F) << 8) | responseBuf[3])
        #yCoord = yCoord >> 2
        print(responseBuf)
        print("Mode: "+str(mode))
        print("X Y: "+str(xCoord)+"  "+str(yCoord))

        #JJ3290616 - Third read
        responseBuf = calibrate.touchDev.read(calibrate.readEP, 15, 5000)
        mode = int(responseBuf[0])
        xCoord = int (((responseBuf[2] & 0x0F) << 8) | responseBuf[1])
        #xCoord = xCoord >> 2
        yCoord = int (((responseBuf[4] & 0x0F) << 8) | responseBuf[3])
        #yCoord = yCoord >> 2
        print(responseBuf)
        print("Mode: "+str(mode))
        print("X Y: "+str(xCoord)+"  "+str(yCoord))

        #JJ3290616 - Fourth read
        responseBuf = calibrate.touchDev.read(calibrate.readEP, 15, 5000)
        mode = int(responseBuf[0])
        xCoord = int (((responseBuf[2] & 0x0F) << 8) | responseBuf[1])
        #xCoord = xCoord >> 2
        yCoord = int (((responseBuf[4] & 0x0F) << 8) | responseBuf[3])
        #yCoord = yCoord >> 2
        print(responseBuf)
        print("Mode: "+str(mode))
        print("X Y: "+str(xCoord)+"  "+str(yCoord))

        #JJ3290616 - Fifth read
        responseBuf = calibrate.touchDev.read(calibrate.readEP, 15, 5000)
        mode = int(responseBuf[0])
        xCoord = int (((responseBuf[2] & 0x0F) << 8) | responseBuf[1])
        #xCoord = xCoord >> 2
        yCoord = int (((responseBuf[4] & 0x0F) << 8) | responseBuf[3])
        #yCoord = yCoord >> 2
        print(responseBuf)
        print("Mode: "+str(mode))
        print("X Y: "+str(xCoord)+"  "+str(yCoord))
        
        time.sleep(10)
"""

if __name__ == "__main__":
    global responseBuf
    print("AR1100 Touch Screen")
    #dummyTest()
    countDown = 3  #45
    returnVal = -1
    cross = 0
    #print("float sz: ", sys.float_info)
    #clck1 = time.clock()
    #tm = time.time()
    act = LCDCalibration()
    #clck2 = time.clock()
    if act.activateCalibration() == 1:
        print("Calibrate command failed")
    else:
        print("Calibrate command success")
        while countDown > 0:
            print("Time to go: "+str(countDown)+" seconds")
            countDown = countDown - 1
            time.sleep(1)

        #for i in range (9):
        while cross < 9:
            #JJ5110816 - Host present hair cross for user to press
            #JJ5110816 - Question: How can the host know when to read after
            #JJ5110816 - presenting the hair cross?
            #time.sleep(0.5)
            #startAt = time.time()  #JJ1071116 - Starting time
            print("Point: ", cross+1)
            returnVal = act.getTouchResponse()
            #finishAt = time.time() #JJ1071116 - Ending time
            #timeTaken = finishAt - startAt
            #print("time take: ",timeTaken)
            if returnVal == 1:  #JJ5120816 - Proceed to next cross
                cross = cross + 1
            elif returnVal == 0:  #JJ5120816 - Time out, stop the calibration
                print("Calibration not complete")
                break
            elif returnVal == 2:  #JJ2081116 - Move too fast, redo
                print("Too fast, redo")
                cross = 10

            if cross == 6:
                cross = 10  #JJ1071116 - Force stop

#            while True:
#                try:
                    #JJ5110816 - Device response to user touch
#                    responseBuf = act.touchDev.read(act.readEP, 15, 5000)
#                    if responseBuf[2] == 0x00:
#                        break;
#                except:
#                    print("Time out: user has not touched the screen")
#                    print("Point: "+str(i+1))
#            print(responseBuf)

    act.exitCalibrationMode()
    print("Calibration terminate")


#JJ3270416 - Beep
#for i in range (1, 4):
    #subprocess.call(["aplay", "bleep_01.wav"])
    #subprocess.call(["aplay", "Electronic_Chime.wav"])
#    subprocess.call(["aplay", "Beep.wav"])
#    time.sleep(0.5)


#JJ3270416 - End of File
