#/usr/bin/env python

#************************************************************************************
#
# FILE     : PMI_ADC.py
# DATE     : Thur, June 23, 2016
# PROLOGUE : This file implements the PMI ADC class which is the API 
#            between a Python script and the BARDA applications.
# CPU TYPE : ARM Cortec - A53 with BMC2837 core
# PROJECT  : iART
# AUTHOR   : JJ
# VERSION  : ??????
# HISTORY  :
#   JJ4230616 - Start coding:
#   JJ4230616 - Create PMI_ADC class definition
#   JJ3180516 - Add readFromBattGasGaugeReg() method
#   JJ3180516 - Add writeToRTCRegister() method
#   JJ3180516 - Add getBatteryDeviceStatus() method
#   JJ3180516 - Add getBatteryVoltage() method
#   JJ3180516 - Add getBatteryCurrent() method
#   JJ3180516 - Add getBatteryTemperature() method
#************************************************************************************

# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

#from __future__ import division

import ctypes
#import struct
import time
import sys
import smbus     #JJ4230616 - For I2C protocol
#import usb.core  #JJ2190416 - Testing USB
#import usb.util  #JJ2190416 -
import RPi.GPIO as GPIO  #JJ4230616 - Import GPIO class


# GPIO pin usage on the Rev 1.0 PCB.  Note that these are PHYSICAL pin numbers on the header and not Broadcom pins
# FPGA configuration pins

# I2C interface
SDA = 3
SCL = 5
# SPI interface 
#JJ4050516 SPI_MOSI = 19
#JJ4050516 SPI_MISO = 21
#JJ4050516 SPI_SCLK = 23
#JJ4050516 SPI_SS_N = 24

# FPGA Register Map
#JJ4050516 REG_FPGA_VERSION = 0x00
#JJ4050516 REG_PCB_VERSION = 0X01

# SPI Bus literals
#JJ4050516 SPI_READ = 0x80
#JJ4050516 SPI_WRITE = 0x00

FPGA_CLK_RATE = 125000000.0

# Motor parameters

# Masks for manipulating the motor control register

#JJ2170516 - Create a class by the name of BatteryManager
#JJ2170516 - Methods will be as follow: Set register address, Read device register, Write device register

#PMI_ADC_ADDR = 0x14  #JJ5240616 - Based on schematic
PMI_ADC_ADDR = 0x26    #JJ4230616 - LTC2485 Input Current Cancellation
PMI_ADC_WR = 0x4C   #JJ5240616 - 01001100
PMI_ADC_RD = 0x4D   #JJ5240616 - 01001101
#I2CBus = 0
#I2CBus = smbus.SMBus(1)
#I2CBus.open(1)  #JJ4020616 - No different

#JJ4230616 - Below are LTC2485 Configuration Bit
SPD = 0x01  #JJ4230616 - Speed Mode. 0 - Auto calibration, 1 - Speed 2X
FB = 0x02  #JJ4230616 - Rejection mode. 1 - 50Hz Rejection
FA = 0x04  #JJ4230616 - Rejection mode. 1 - 60Hz Rejection
IM = 0x08  #JJ4230616 - Temperature Sensoe. 0 - External Input, 1 - Temperature Input

OPER_MODE = SPD

byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]) #JJ2170516 - I/O byte buffer
#outBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2170516 - Output buffer to write to EEPrompt
#inBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])   #JJ2170516 - Input buffer to contain page read from EEPrompt

#inputData = int(0)
tempMSB = 0
tempLSB = 0


#JJ4230616 - Soft_I2C Class ----------------------------------------------------
#JJ4230616 - Software I2C Class definition
class Soft_I2C:
    """ JJ4230616 - This class implement software I2C functionality """
    def __init__(self):
        #JJ4230616 - First configure GPIO pin for SDA and SCL
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SDA, GPIO.OUT)  #JJ4230616 - Start as output, but is both I/O
        GPIO.setup(SCL, GPIO.OUT)  #JJ4230616 - Always as output
        self.freeBus()

    def exitI2C(self):
        print("Inside exitI2C()...")
        #data = GPIO.gpio_function(SDA)
        #clock = GPIO.gpio_function(SCL)
        #print("B4: "+str(data)+str(clock))
        GPIO.cleanup()
        I2CBus = smbus.SMBus(1)
        I2CBus.close()
        #GPIO.setmode(GPIO.BOARD)
        #GPIO.I2C
        #GPIO.I2C
        #print("4B: "+str(data)+str(clock))

    #JJ4230616 - Set both data and clock high to indicate bus is free
    def freeBus(self):
        GPIO.output(SDA, True)
        GPIO.output(SCL, True)

    def I2CDelay(self, lapse):
        for i in range(0, lapse):
            pass

    #JJ4230616 - Data high to low when clock is high
    def I2CStart(self):
        GPIO.output(SDA, True)  #JJ4230616 - Data high
        GPIO.output(SCL, True)  #JJ4230616 - Clock high
        self.I2CDelay(50)
        GPIO.output(SDA, False) #JJ4230616 - Data low
        self.I2CDelay(50)        
        GPIO.output(SCL, False)  #JJ4230616 - Clock low
        GPIO.output(SDA, False)  #JJ4230616 - Data low
        self.I2CDelay(50)

    #JJ4230616 - Data low to high when clock is high
    def I2CStop(self):
        GPIO.output(SDA, False)  #JJ4230616 - Data low
        GPIO.output(SCL, False)  #JJ4230616 - Clock low
        self.I2CDelay(50)
        GPIO.output(SCL, True)  #JJ4230616 - Clock high
        self.I2CDelay(50)
        GPIO.output(SDA, True)  #JJ4230616 - Data high
        self.I2CDelay(50)
        self.freeBus()  #JJ5240616 - 

    #JJ4230616 - Master acknowledge to slave
    def I2CMasterAck(self, direction):
        if direction == 1:
            GPIO.output(SDA, True)  #JJ5240616 - Stop reading; master NAK
        else:
            GPIO.output(SDA, False) #JJ5240616 - Master ACK
        self.I2CDelay(50)
        GPIO.output(SCL, True)  #JJ4230616 - Clock high
        self.I2CDelay(50)
        GPIO.output(SCL, False)  #JJ4230616 - Clock low
        self.I2CDelay(50)

    #JJ4230616 - Read Acknowledgement from slave
    #JJ5240616 - Return 0 when ok; 1 when not ok
    def I2CCheckSlaveAck(self):
        trial = 100
        retAck = 1
        GPIO.setup(SDA, GPIO.IN)  #JJ5240616 - Change SDA to input mode
        self.I2CDelay(40)
        for i in range (0, trial):
            retAck = GPIO.input(SDA)
            if retAck == 0:
                break
        #JJ5240616 - Here we generate the ninth clock cycle
        self.I2CDelay(50)
        GPIO.output(SCL, True)  #JJ5240616 - Clock high
        self.I2CDelay(50)
        GPIO.output(SCL, False)  #JJ5240616 - Clock low
        self.I2CDelay(50)
        #JJ5240616 - Take note: may need to move down the read into here
        GPIO.setup(SDA, GPIO.OUT) #JJ5240616 - Change SDA back to output mode
        return retAck

    #JJ5240616 - Write data to slave
    #JJ5240616 - Return 0 when ok; -1 when not ok
    def I2CWrite(self, numOfByte):
        global byteBuffer
        tempVal = 0
        retFlag = 0
        ack = 0
        i = 0
        #j = 0

        #JJ5240616 - This loop thru the entire byte buffer
        while (i < numOfByte):  #JJ5240616 - May consider to include (i < maxSize)
            tempVal = byteBuffer[i]
            #JJ5240616 - This loop thru the 8 bits of the byte
            for j in range(0, 8):
                GPIO.output(SCL, False)  #JJ5240616 - Clock low

                if (tempVal >> (7 - j)) & 0x01:
                    GPIO.output(SDA, True)  #JJ5240616 - SDA high
                else:
                    GPIO.output(SDA, False) #JJ5240616 - SDA low

                self.I2CDelay(40)
                #JJ5240616 - Now generate clock pulse
                GPIO.output(SCL, True)  #JJ5240616 - Clock high
                self.I2CDelay(40)
                GPIO.output(SCL, False)  #JJ5240616 - Clock low
                self.I2CDelay(30)

            ack = self.I2CCheckSlaveAck()  #JJ5240616 - Get acknowledgement from slave
            if ack == 1:  #JJ5240616 - Slave is busy
                retFlag = -1
                break
            else:
                self.I2CDelay(30)
                i = i + 1

        return retFlag

    #JJ5240616 - This method reads in bits send out from slave
    def I2CReadByte(self):
        tempData = 0x00
        for j in range (0, 8):
            GPIO.output(SCL, True)  #JJ5240616 - Clock high
            self.I2CDelay(40)

            #JJ5240616 - Read SDA pin
            if GPIO.input(SDA) == 1:
                tempData = tempData | (0x80 >> j)

            GPIO.output(SCL, False)  #JJ5240616 - Clock low
            self.I2CDelay(40)

        return tempData

    #JJ5240616 - Read data from slave
    #JJ5240616 - Return 0 when ok; -1 when not ok
    def I2CRead(self, NumOfByte):
        global byteBuffer
        retFlag = 0
        byteBuffer[0] = PMI_ADC_RD
        self.I2CStart()  #JJ5240616 - Start I2C communication
        retFlag = self.I2CWrite(1)
        if retFlag == 0:  #JJ5240616 - Write to slave successfull
            self.I2CDelay(60)
            #JJ5240616 - If we reach here, slave already acknowledged
            byteBuffer[0] = 0x00
            for j in range (0, NumOfByte):
                GPIO.setup(SDA, GPIO.IN)  #JJ5240616 - Change SDA to input mode
                byteBuffer[j] = self.I2CReadByte()
                GPIO.setup(SDA, GPIO.OUT) #JJ5240616 - Change SDA back to output mode

                if j < (NumOfByte - 1):
                    self.I2CMasterAck(0)
                else:
                    self.I2CMasterAck(1)
                self.I2CDelay(40)

            #self.I2CStop()
        #else:
        self.I2CStop()
        return retFlag

#JJ4230616 - Soft_I2C Class END ------------------------------------------------


#JJ4230616 - PMI ADC Class definition
class PMI_ADC:
    """ JJ4230616 - This class provide API for Input Current Cancellation LTC2485 device """
    """ set ??? """
    I2CBus = 0

    def __init__(self):
        global OPER_MODE
        self.I2CBus = Soft_I2C()  #JJ5240616 - Create Software I2C object
        pass

    def configureDevice(self, newMode):
        """ JJ4230616 - This method write operation mode to configure register """
        global byteBuffer
        global OPER_MODE
        retValue = 0
        OPER_MODE = newMode
        byteBuffer[0] = PMI_ADC_WR  #JJ5240616 - Slave address with R/W set 0
        byteBuffer[1] = OPER_MODE   #JJ5240616 - Data to slave device
        self.I2CBus.I2CStart()
        if self.I2CBus.I2CWrite(2) == 0:  #JJ5240616 - Write ok
            retValue = 0
        self.I2CBus.I2CStop()
        return retValue

    def readByteFromDevice(self):
        """ JJ4020616 - This method reads 1 byte of data from slave device """
        global byteBuffer
        inputData = 0x0000
        #PMI_ADC_WR
        self.I2CBus.I2CRead(1)
        inputData = byteBuffer[0]
        return inputData

    def readOutputData(self):
        """ JJ4230616 - This method read 4 bytes of data from LTC2485 output register """
        global byteBuffer
        #global OPER_MODE
        readStatus = 0
        negValue = 0
        longData = 0x00000000
        readStatus = self.I2CBus.I2CRead(4)
        if readStatus == 0:
            #JJ5240616 - Further computation is needed here
            statusBits = byteBuffer[0] & 0xC0
            longData = (byteBuffer[0] & 0x3F)  #JJ5240616 - Annex MSB
            longData = (longData << 8) | byteBuffer[1]  #JJ5240616 - Annex byte 2
            longData = (longData << 8) | byteBuffer[2]  #JJ5240616 - Annex byte 1
            longData = (longData << 8) | byteBuffer[3]  #JJ5240616 - Annex LSB
            longData = longData >> 6  #JJ5240616 - Don't want to compute the fractional part
            if statusBits & 0x40:  #JJ5240616 - Result is negative, flip the result
                longData = ~longData
                longData = longData & 0xFFFFFFFF
                longData = longData + 1
                negValue = 1
            #JJ5240616 - For now just report the value.
            #JJ5240616 - Will apply the formula to data and return the result
            longData = longData * 5.0 / 0x80000000
            if negValue == 1:
                longData = longData * -1

        return readStatus, longData

    def quit(self):
        """ JJ5240616 - This method did a clean up on GPIO """
        self.I2CBus.exitI2C()

    def configureBatteryManager(self):
        """ JJ2170516 - This method is reserved for codes to configure gas gauge device. """
        """ JJ2170516 - Don't think we need to facilitate RTC configuration. """
        """ JJ2170516 - As such, leave this method blank to preserve for future use. """
        pass

    def getBatteryVoltage(self):
        """ JJ3180516 - This method returns battery voltage to calling method """
        #JJ3180516 - This method had been step thru with exception of I2C function;
        #JJ3180516 - so far the logic seems to be correct.
        #JJ3180516 - Here we set ADC mode to single conversion
        global byteBuffer
        retVoltage = 0x0000
        return retVoltage


if __name__ == "__main__":
    #N = 2
    #addr = '0x7E200000'
    #g = (ctypes.c_int*N).from_address(int(addr, 16))
    #print(g[0], g[1])
    pmiADC = PMI_ADC()
    pmiADC.configureDevice(OPER_MODE)
    #while True:
    for i in range (0, 3):
        #current = pmiADC.readByteFromDevice()
        status, volt = pmiADC.readOutputData()
        print("Status: " + str(status))
        print("Voltage: " + str(volt), "\n")
        #print("Current: " + str(current), "\n")
        #print("Temperature: " + str(temperature), "\n")
        time.sleep(3)
    pmiADC.quit()
    print("Exit test")
    #battManager.getBattVoltage()


#JJ EEPromptPage = (501 << 6) + 0x00
#JJ outBuffer.append(EEPromptPage >> 8)  #JJ4140416 - MSB of EEPrompt internal address
#JJ outBuffer.append(EEPromptPage & 0x00FF)  #JJ4140416 - LSB of EEPrompt internal address
#bus.write_i2c_block_data(DEVICE_ADDRESS, outBuffer[0], outBuffer[1:])
#JJ4140416 - Next we need to read in byte by byte the next 64 bytes of a page
#for i in range (1, 64):
#    inBuffer.append(bus.read_byte(DEVICE_ADDRESS))


#JJ1250416 - Below code added from PyUSB/Mailing Lists


#time.sleep(1)



#JJ2170516 ----- End of BatteryManager.py file -----
