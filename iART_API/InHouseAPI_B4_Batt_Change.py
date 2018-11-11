#/usr/bin/env python

#*******************************************************************************
#
# FILE     : InHouseAPI.py
# DATE     : Tue, May 17, 2016
# PROLOGUE : This file implements in house class which is the API between a
#            Python script and the BARDA applications.
# CPU TYPE : ARM Cortec - A53 with BMC2837 core
# PROJECT  : iART
# AUTHOR   : JJ
# VERSION  : ??????
# HISTORY  :
#   JJ3110516 - Start coding Backlight_PWM:
#   JJ3110516 - Create Backlight_PWM class definition
#   JJ3110516 - Add adjustDutyCycle method
#
#   JJ4120516 - Start coding RTC:
#   JJ4120516 - Add method to set / get date on RTC device
#   JJ5130516 - Add method to set / get time on RTC device
#
#   JJ1300516 - Start coding Buzzer:
#   JJ1300516 - Create Buzzer class definition
#   JJ1300516 - Add turnOn method to activate buzzer to a specific duty cycle
#
#   JJ2170516 - Start coding Battery management:
#   JJ2170516 - Create BatteryManager class definition
#   JJ3180516 - Add readFromBattGasGaugeReg() method
#   JJ3180516 - Add getBatteryDeviceStatus() method
#   JJ3180516 - Add getBatteryVoltage() method
#   JJ3180516 - Add getBatteryCurrent() method
#   JJ3180516 - Add getBatteryTemperature() method
#
#   JJ4190516 - Start coding Ambient sensor:
#   JJ4190516 - Create AmbientSensor class definition
#   JJ4190516 - Add method to read lux value, getLux()
#   JJ5100616 - Start testing codes with evaluation kit
#
#   JJ2070616 - Start coding Current, Voltage, Power monitor:
#   JJ2070616 - Create PowerMonitor class definition
#   JJ3080616 - Add method to set and get configuration register
#   JJ3080616 - Add method to get Shunt Voltage data from reg 0x01
#   JJ3080616 - Add method to get Bus Voltage data from reg 0x02
#   JJ3080616 - Add method to get Power data from reg 0x03
#   JJ3080616 - Add method to get Current data from reg 0x04
#
#   JJ2140616 - Create General puspose class definition; GeneralPurpose()
#   JJ2140616 - Add method to detect home switch status; detectHomeSwitch()
#   JJ2140616 - Add method to detect door switch status: detectDoorSwitch()
#   JJ2140616 - Add method to detect disposable presence; disposableStatus()
#   JJ2140616 - Add method to detect the occurene of collision; collisionStatus()
#   JJ2140616 - Add method to turn on and off the sensor; turnOnSensor(); turnOffSensor()
#   JJ3150616 - Add method to detect FPGA_INIT_B pin status
#   JJ3150616 - Add method to detect FPGA_DONE pin status
#
#   JJ3130716 - Change DOOR_SW from pin 26 to pin 33. Pin 26 now being used as
#               chip select for ADS8320
#
#*******************************************************************************

# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

#from __future__ import division

#import re
#import struct
import barda  #JJ1250716 - For testing barda set enable method
import os  #JJ2190716 - Need it to help clear screen
import time
import sys
import smbus     #JJ4140416 - For I2C protocol
#import usb.core  #JJ2190416 - Testing USB
#import usb.util  #JJ2190416 -
import RPi.GPIO as GPIO  #JJ2260416 - Import GPIO class


# GPIO pin usage on the Rev 1.0 PCB.  Note that these are PHYSICAL pin numbers on the header and not Broadcom pins
# FPGA configuration pins

# I2C interface
I2C_SDA = 3
I2C_SCL = 5
I2CBus = smbus.SMBus(1)
# SPI interface 
#SPI_MOSI = 19
#SPI_MISO = 21
#SPI_SCLK = 23
#SPI_SS_N = 26

# FPGA Register Map
#JJ4050516 REG_FPGA_VERSION = 0x00
#JJ4050516 REG_PCB_VERSION = 0X01

# SPI Bus literals
#JJ4050516 SPI_READ = 0x80
#JJ4050516 SPI_WRITE = 0x00

#FPGA_CLK_RATE = 125000000.0

#JJ2170516 - Global variables for I2C use
tempMSB = 0
tempLSB = 0

byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]) #JJ4120516 - I/O byte buffer
outBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ4140416 - Output buffer to write to EEPrompt
#inBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])   #JJ4140416 - Input buffer to contain page read from EEPrompt


#JJ3110516 - Global variables for PWM
dutyCycle = 100  #JJ3110516 - Assuming default is 100%
BLight_pin = 12

#JJ3110516 - Global variables for Buzzer
buzzerDutycycle = 50  #JJ3110516 - Assuming default is 40%
buzzer_pin = 31
#buzzer_PWM = 0  #JJ1300516 - KIV. Maybe used in later stage of project development

#JJ5030616 - Global variables for Battery Management class
BATT_ADDR = 0x64    #JJ2170516 - LTC2943 Battery gas gauge Device
BATT_ADDR_WR = 0xC8 #JJ2170516 - 11001000
BATT_ADDR_RD = 0xC9 #JJ2170516 - 11001001

CHARGE_ENABLE = 36  #JJ2140616 - No need to do anything at the moment
AC_ADAPTER_PIN = 38

#JJ2210616 - Below variables are used for manipulating status register
UVLO_ALERT = 0x01       #JJ2210616 - 00000001
VOLTAGE_ALERT = 0x02    #JJ2210616 - 00000010
CHRG_ALERT_LOW = 0x04   #JJ2210616 - 00000100
CHRG_ALERT_HIGH = 0x08  #JJ2210616 - 00001000
TEMP_ALERT = 0x10       #JJ2210616 - 00010000
ACR_ALERT = 0x20        #JJ2210616 - 00100000
CURRENT_ALERT = 0x40    #JJ2210616 - 01000000

#JJ3180516 - Below are LTC2943 register addresses
STATUS_REG = 0  #JJ3180516 - Reg val represent device status
CONTROL_REG = 1  #JJ3180516 - Reg val control the device's operation
ACR_MSB = 2  #JJ3180516 - Accumulated Charge Register
ACR_LSB = 3  #JJ3180516 - Accumulated Charge Register
CHG_TRHOLD_HIGH_MSB = 4  #JJ3180516 - Charge Threshold High
CHG_TRHOLD_HIGH_LSB = 5  #JJ3180516 - Charge Threshold High
CHG_TRHOLD_LOW_MSB = 6  #JJ3180516 - Charge Threshold Low
CHG_TRHOLD_LOW_LSB = 7  #JJ3180516 - Charge Threshold Low
VOLTAGE_MSB = 8  #JJ3180516 - Voltage
VOLTAGE_LSB = 9  #JJ3180516 - Voltage
V_TRHOLD_HIGH_MSB = 10  #JJ3180516 - Voltage Threshold High
V_TRHOLD_HIGH_LSB = 11  #JJ3180516 - Voltage Threshold High
V_TRHOLD_LOW_MSB = 12  #JJ3180516 - Voltage Threshold Low
V_TRHOLD_LOW_LSB = 13  #JJ3180516 - Voltage Threshold Low
CURRENT_MSB = 14  #JJ3180516 - Reg val represents charge current
CURRENT_LSB = 15  #JJ3180516 - Reg val represents charge current
CUR_TRHOLD_HIGH_MSB = 16  #JJ3180516 - Current Threshold High
CUR_TRHOLD_HIGH_LSB = 17  #JJ3180516 - Current Threshold High
CUR_TRHOLD_LOW_MSB = 18  #JJ3180516 - Current Threshold Low
CUR_TRHOLD_LOW_LSB = 19  #JJ3180516 - Current Threshold Low
TEMPERATURE_MSB = 20  #JJ3180516 - Temperature
TEMPERATURE_LSB = 21  #JJ3180516 - Temperature
TEMP_TRHOLD_HIGH = 22  #JJ3180516 - Temperature Threshold High
TEMP_TRHOLD_LOW = 23  #JJ3180516 - Temperature Threshold Low

#JJ5100616 - Global variables for Ambient sensor
AMBIENT_ADDR = 0x4A   #JJ4180516 - MAX44009 Ambient Light Sensor Device (If A0 connect to ground)
#AMBIENT_ADDR = 0x4B   #JJ4180516 - MAX44009 Ambient Light Sensor Device (If A0 connect to VCC)

#JJ4190516 - Below are MAX44009 register addresses
INT_STAT = 0  #JJ4190516 - Reg val represents interrupt status
INT_EN = 1  #JJ4190516 - Reg val represents interrupt enable
CFG_REG = 2  #JJ4190516 - Reg val represents device's configuration setting
LUX_MSB = 3  #JJ4190516 - Reg val represents exponent and MSB of mantissa for lux value
LUX_LSB = 4  #JJ4190516 - Reg val represents LSB of mantissa for lux value
UPR_TRHOLD = 5  #JJ4190516 - Reg val represents upper threshold MSB
LWR_TRHOLD = 6  #JJ4190516 - Reg val represents lower threshold LSB
TRHOLD_TMR = 7  #JJ4190516 - Reg val represents threshold timer

UPPER_THRESHOLD = 500  #JJ5200516 - To be advised later
LOWER_THRESHOLD = 1    #JJ5200516 - To be advised later

DOOR_CLOSE_CONST = 0.045  #JJ2140616 - TBD. At the moment 0.045 is the lowest the chip can sense

#JJ5100616 - Global variables for Power Monitor class
POW_MONITOR_ADDR = 0x40   #JJ2070616 - INA220 Power monitoa Device (A0, A1 connect to ground)

#JJ2070616 - Below are INA220 Register (Command) Set
INA_CONFIG_REG = 0x00  #JJ2070616 - Configuration register
SHUNT_VOLT_REG = 0x01  #JJ2070616 - Register contains shunt drop voltage
BUS_VOLT_REG = 0x02  #JJ2070616 - Register contains bus drop voltage
POWER_REG = 0x03  #JJ2070616 - Register contains power; only if calibration reg is set
CURRENT_REG = 0x04  #JJ2070616 - Register contains current; only if calibration reg is set
CALIBRATION_REG = 0x05  #JJ2070616 - Register contains calibration data

#JJ2070616 - Below is configuration register bit format
INA_MODE1 = 0x0001
INA_MODE2 = 0x0002
INA_MODE3 = 0x0004
INA_SADC1 = 0x0008
INA_SADC2 = 0x0010
INA_SADC3 = 0x0020
INA_SADC4 = 0x0040
INA_BADC1 = 0x0080
INA_BADC2 = 0x0100
INA_BADC3 = 0x0200
INA_BADC4 = 0x0400
INA_PG0 = 0x0800
INA_PG1 = 0x1000
INA_BRNG = 0x2000
INA_UNUSED = 0x4000
INA_RST = 0x8000

#JJ3080616 - GAIN setting permutation
GAIN_40mV = 0x0000
GAIN_80mV = INA_PG0
GAIN_160mV = INA_PG1
GAIN_320mV = INA_PG0 | INA_PG1
#GAIN_USED = GAIN_40mV
GAIN_USED = GAIN_320mV  #JJ5170616 - Test

#JJ3080616 - Bus Voltage Range setting permutation
BUS_16V = 0x0000
BUS_32V = INA_BRNG
#BUS_RANGE_USED = BUS_16V
BUS_RANGE_USED = BUS_32V  #JJ5170616 - Test

#JJ3080616 - Default config setting after POR is 0x399F
INA_DEFAULT_CONFIG = 0x019F  #JJ2070616 - 
#UPPER_THRESHOLD = 40.50  #JJ3250516 - To be advised later
#LOWER_THRESHOLD = -5.50  #JJ3250516 - To be advised later


# Motor parameters

# Masks for manipulating the motor control register

#JJ4050516 - Global variables for RTC
RTC_ADDR = 0x68   #JJ4050516 - DS1337 RTC Device
RTC_ADDR_WR = 0xD0 #JJ4050516 - 11010000
RTC_ADDR_RD = 0xD1 #JJ4050516 - 11010001

#JJ5060516 - Below are RTC register addresses
SS_ADDR = 0  #JJ5060516 - Reg val represent seconds
MN_ADDR = 1  #JJ5060516 - Reg val represents minutes
HH_ADDR = 2  #JJ5060516 - Reg val represents hour
DY_ADDR = 3  #JJ5060516 - Reg val represent day 1 to 7 for Monday to Sunday
DD_ADDR = 4  #JJ5060516 - Reg val represents date
MM_ADDR = 5  #JJ5060516 - Reg val represents month
YY_ADDR = 6  #JJ5060516 - Reg val represents year
#JJ5060516 - Though the RTC device supports alarm function, but don't think we are going to
#JJ5060516 - use this feature in our project. As such, below var are temporarily blocked
#AL1_SS_ADDR = 7
#AL1_MN_ADDR = 8
#AL1_HH_ADDR = 9
#AL1_DT_ADDR = 10
#AL2_MN_ADDR = 11
#AL2_HH_ADDR = 12
#AL2_DT_ADDR = 13
CNTRL_REG = 14  #JJ5060516 - Reg val represents control data
STAT_REG = 15  #JJ5060516 - Reg val represents device status


#JJ2140616 - Global variables for General Purpose class
FPGA_INIT_B = 11
FPGA_DONE = 13
EXIT_SW_PIN = 15
HOME_SW_PIN = 16
DOOR_SW_PIN = 33  #JJ3130716 - Change from pin 26 to 33; pin 26 is now being used as chip select for ADS8320
DISPOSABLE_SW_PIN = 29
COLLISION_PIN = 37
#SENSOR_ENABLE_PIN = 40

#JJ1220816 - Global variables for mode 1 test class
MODE1_HOLD = 32
MODE1_TICK = 35
MODE1_INT_CTRL = 40

#JJ2140616 - This section for individual method --------------------------------


#JJ2140616 - END of individual method section ----------------------------------


#JJ4050516 - RTC class start ---------------------------------------------------
#JJ4050516 - Methods will be as follow: Set register address, Read device register, Write device register
#JJ4050516 - RTC Class definition
class iART_RTC:
    """ JJ4050516 This class provide API for RTC device DS1337 to configure its register, """
    """ set date, get date."""

    def __init__(self):
        #I2CBus = smbus.SMBus(1)
        pass

    def readFromRTCRegister(self, size):
        """ JJ3220616 - This method reads RTC register from register 0 for the amount of """
        """ JJ3220616 - bytes stated in size """
        """ JJ2020816 - Return 0 if not error, 1 if there is error """
        global byteBuffer  #JJ3220616 -
        byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        #for j in range(0, 7):
        #    byteBuffer[j] = 0
        readStatus = 0
        try:
            byteBuffer = I2CBus.read_i2c_block_data(RTC_ADDR, SS_ADDR, size)
        except:
            readStatus = -1
            print("readFromRTCRegister ends with error")
        return readStatus

    def writeToRTCRegister(self, startAddr, numByte):
        """ JJ2020816 - This method writes to RTC register from startAddr """
        """ JJ2020816 - Return 0 if not error, 1 if there is error """
        global byteBuffer  #JJ3220616 - 
        outList = list(byteBuffer[0:numByte])
        writeStatus = 0
        try:
            #I2CBus.write_i2c_block_data(RTC_ADDR, startAddr, byteBuffer)
            I2CBus.write_i2c_block_data(RTC_ADDR, startAddr, outList)
        except:
            writeStatus = -1
            print("writeToRTCRegister ends with error")
        return writeStatus

    def getDeviceStatus(self):
        """ JJ5060516 - This method return the RTC device status."""
        """ JJ5060516 - 0 - Device is ok; 1 - Device is not initialised yet. """
        global byteBuffer  #JJ3220616 -
        if self.readFromRTCRegister(16) == 0:  #JJ3220616 - Read ok
            retValue = (byteBuffer[STAT_REG] & 0x80) >> 7  #JJ2170516 - Bit 7 of status register
        else:  #JJ3220616 - Read not ok
            retValue = -1
        return retValue

    def configureRTC(self):
        """ JJ2170516 - This method is reserved for codes to configure RTC device. """
        """ JJ2170516 - Don't think we need to facilitate RTC configuration. """
        """ JJ2170516 - As such, leave this method blank to preserve for future use. """
        global outBuffer
        retValue = 0
        newConfig = 0x80  #JJ3220616 - This turn on OSF bit and set A2F and A1F to 0
        try:
            #I2CBus.write_byte_data(RTC_ADDR, STAT_REG, newConfig)
            pass
        except:
            retValue = -1
        return retValue

    def startRTC(self):
        """ JJ4250816 - This method starts RTC device. """
        """ JJ4250816 -  """
        """ JJ4250816 -  """
        global outBuffer
        retValue = 0
        newConfig = 0x00  #JJ4250816 - This turn EOSC to 0
        try:
            I2CBus.write_byte_data(RTC_ADDR, CNTRL_REG, newConfig)
            pass
            I2CBus.write_byte_data(RTC_ADDR, STAT_REG, newConfig)
        except:
            retValue = -1
        return retValue

    def getDate(self):
        global byteBuffer  #JJ3220616 - 
        """
        if self.readFromRTCRegister(7) == -1:  #JJ2020816 - Clear buffer
            byteBuffer[0] = 0
            byteBuffer[1] = 0
            byteBuffer[2] = 0
            byteBuffer[3] = 0
            byteBuffer[4] = 0
            byteBuffer[5] = 0
            byteBuffer[6] = 0
        """
        self.readFromRTCRegister(7)
        retDay = byteBuffer[DY_ADDR] & 0x07  #JJ5060516 - Bit 0 to 2 represent day. We don't use it for our project
        tempValue = ((byteBuffer[DD_ADDR] >> 4) & 0x03) * 10
        retDate = tempValue + (byteBuffer[DD_ADDR] & 0x0F)
        tempValue = ((byteBuffer[MM_ADDR] >> 4) & 0x01) * 10
        retMonth = tempValue + (byteBuffer[MM_ADDR] & 0x0F)
        tempValue = ((byteBuffer[YY_ADDR] >> 4) & 0x0F) * 10
        retYear = 2000 + (tempValue + (byteBuffer[YY_ADDR] & 0x0F))
        #JJ4230616 - Below string's format is DDMMYYYY (DayMonthYear)
        retDDMMYYYY = str(retDate).zfill(2) + str(retMonth).zfill(2) + str(retYear).zfill(4)
        #return retDate, retMonth, retYear
        return retDDMMYYYY

    #def setDate(self, dd, mm, yy):
    def setDate(self, dateString):
        global byteBuffer  #JJ3220616 - 
        global tempMSB
        global tempLSB
        #tempMSB = 0
        #tempLSB = 0
        byteBuffer[0] = 0  #JJ4120516 - Day
        #JJ4120516 - Here we convert date to its BCD format
        #dateString = "22.06.2016"
        dd = int(dateString[0:2])  #JJ3220616 - Extract Day
        mm = int(dateString[3:5])  #JJ3220616 - Extract Month
        yy = int(dateString[8:10])  #JJ3220616 - Extract Year
        #yy = yy % 100  #JJ3220616 - We only need the last two digit of the year
        tempMSB = int(dd / 10)
        tempLSB = int(dd % 10)
        byteBuffer[1] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ4120516 - Date in BCD format
        #JJ4120516 - Here we convert month to its BCD format
        tempMSB = int(mm / 10)
        tempLSB = int(mm % 10)
        byteBuffer[2] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ4120516 - Month in BCD format
        #JJ4120516 - Here we convert year to its BCD format
        tempMSB = int(yy / 10)
        tempLSB = int(yy % 10)
        byteBuffer[3] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ4120516 - Year in BCD format
        self.writeToRTCRegister(DY_ADDR, 4)  #JJ1160516 - Official

    def getTime(self):
        global byteBuffer  #JJ3220616 - 
        self.readFromRTCRegister(6)  #JJ1160516 - Official code
        """
        if self.readFromRTCRegister(6) == -1:  #JJ2020816 - Clear buffer
            byteBuffer[0] = 0
            byteBuffer[1] = 0
            byteBuffer[2] = 0
            byteBuffer[3] = 0
            byteBuffer[4] = 0
            byteBuffer[5] = 0
            byteBuffer[6] = 0
        """
        upperNibble = ((byteBuffer[SS_ADDR] >> 4) & 0x07) * 10
        retSeconds = upperNibble + (byteBuffer[SS_ADDR] & 0x0F)  #JJ5130516 - This gives seconds
        upperNibble = ((byteBuffer[MN_ADDR] >> 4) & 0x07) * 10
        retMinutes = upperNibble + (byteBuffer[MN_ADDR] & 0x0F)  #JJ5130516 - This gives minutes
        upperNibble = byteBuffer[HH_ADDR] >> 6  #JJ5130516 - Move 12/24 bit to bit 0
        if (upperNibble & 0x01):  #JJ5130516 - 12 hour format is used
            upperNibble = ((byteBuffer[HH_ADDR] >> 4) & 0x01) * 10
            retHours = upperNibble + (byteBuffer[HH_ADDR] & 0x0F)
            #JJ5130516 - Next we determine AM or PM
            upperNibble = byteBuffer[HH_ADDR] >> 5  #JJ5130516 - Move AM/PM bit to bit 0
            #JJ4230616 - JJTODO: To convert time into 24 hour when saved as AM/PM mode
            if (upperNibble & 0x01):
                ampmString = "PM"
                if retHours != 12:
                    retHours += 12
            else:
                ampmString = "AM"
                if retHours == 12:
                    retHours = 0
        else:  #JJ5130516 - 24 hour format is used
            upperNibble = ((byteBuffer[HH_ADDR] >> 4) & 0x03) * 10
            retHours = upperNibble + (byteBuffer[HH_ADDR] & 0x0F)  #JJ5130516 - This gives hours
        #JJ4230616 - Below string's format is HHMNSS (HourMinuteSecond)
        #retTime = str('%2.2u' % retHours) + str('%2.2u' % retMinutes) + str('%2.2u' % retSeconds)
        retTime = str(retHours).zfill(2) + str(retMinutes).zfill(2) + str(retSeconds).zfill(2)
        #return retSeconds, retMinutes, retHours, ampmString  #JJ5130516 - Decide whether to return ampmString as well
        return retTime

    def setTime12H(self, ss, mn, hh, ampm):
        """ JJ3220616 - This method set time in 12 hour format """
        #JJ5130516 - Here we convert seconds to its BCD format
        global byteBuffer  #JJ3220616 - 
        global tempMSB
        global tempLSB
        tempMSB = int(ss / 10)
        tempLSB = int(ss % 10)
        byteBuffer[0] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ5130516 - Seconds in BCD format
        #JJ5130516 - Here we convert minutes to its BCD format
        tempMSB = int(mn / 10)
        tempLSB = int(mn % 10)
        byteBuffer[1] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ5130516 - Minutes in BCD format
        #JJ4120516 - Here we convert hours to its BCD format
        #tempMSB = int(hh / 10)
        tempMSB = 0x04  #JJ5130516 - Turn on 12 hour bit, since we only support 12 hour format
        if (ampm == 1):
            tempMSB = tempMSB | 0x02  #JJ5130516 - Turn on PM bit
        if (hh > 9):
            tempMSB = tempMSB | 0x01  #JJ5130516 - Turn on 10 Hour bit
        tempLSB = int(hh % 10)
        byteBuffer[2] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ5130516 - Hours in BCD format
        self.writeToRTCRegister(SS_ADDR, 3)  #JJ1160516 - This is official code, don't remove

    #def setTime(self, ss, mn, hh):
    def setTime(self, timeString):
        """ JJ3220616 - This method set time in 24 hour format """
        #JJ3220616 - Here we convert seconds to its BCD format
        global byteBuffer  #JJ3220616 - 
        global tempMSB
        global tempLSB
        #timeString = "17.05.20"
        hh = int(timeString[0:2])  #JJ3220616 - Extract Hour
        mn = int(timeString[3:5])  #JJ3220616 - Extract Minute
        ss = int(timeString[6:8])  #JJ3220616 - Extract Second
        tempMSB = int(ss / 10)
        tempLSB = int(ss % 10)
        byteBuffer[0] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ3220616 - Seconds in BCD format
        #JJ3220616 - Here we convert minutes to its BCD format
        tempMSB = int(mn / 10)
        tempLSB = int(mn % 10)
        byteBuffer[1] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ3220616 - Minutes in BCD format
        #JJ3220616 - Here we convert hours to its BCD format
        tempMSB = int(hh / 10)        
        tempLSB = int(hh % 10)
        byteBuffer[2] = (tempMSB << 4) | (tempLSB & 0x0F)  #JJ3220616 - Hours in BCD format
        self.writeToRTCRegister(SS_ADDR, 3)  #JJ3220616 - This is official code, don't remove
        #JJ4250816 - Now that date and time has been set, it is time to start the RTC
        self.startRTC()
#JJ4050516 - RTC class end -----------------------------------------------------


#JJ3110516 - Backlight_PWM Class definition ------------------------------------
class Backlight_PWM:
    """ JJ3110516 - This class provides API to adjust LCD backlight via PWM """

    def __init__(self):
        global dutyCycle
        global BLight_pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BLight_pin, GPIO.OUT)
        self.BLight_PWM = GPIO.PWM(BLight_pin, 1000)  #JJ3110516 - Set to 1KHz freq
        self.BLight_PWM.start(dutyCycle)  #JJ3110516 - Need to know how to control this method
        #self.BLight_PWM.
        #pass

    def adjustDutyCycle(self, dutyCycle):
        if dutyCycle > 100:  #JJ3110516 - Out of range, don't do anything
            pass
        else:
            self.BLight_PWM.ChangeDutyCycle(dutyCycle)
            #time.sleep(0.5)
        #GPIO.cleanup()

    def disableBlightPWM(self, dutyCycle):
        self.BLight_PWM.ChangeDutyCycle(dutyCycle)
        #time.sleep(0.1)
        self.BLight_PWM.stop()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(BLight_pin, GPIO.OUT)
        time.sleep(0.1)
        GPIO.output(BLight_pin, True)
        #time.sleep(5)
#JJ3110516 - Backlight_PWM Class END -------------------------------------------


#JJ1300516 - Buzzer Class definition -------------------------------------------
class Buzzer:
    """ JJ1300516 - This class provides API to activate buzzer via PWM """
    buzzer_PWM = 0

    #JJ4010916 - NOTE: turnOn() method is being used by GUI to same beep sound.
    #JJ4010916 - This method uses PWM to generate sound, but because the dutycycle
    #JJ4010916 - generated is not consistent thereby causing the sound to distort especially
    #JJ4010916 - when repeated sound is necessary. However, if Beep() method is used, the
    #JJ4010916 - sound produced seem to be less distorted.
    #JJ4010916 - So in order not to change GUI application, the turnOn() and Beep() methods
    #JJ4010916 - will be swapped when necessary.
    def __init__(self):
        global buzzer_pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(buzzer_pin, GPIO.OUT)  #JJ4010916 - buzzer_pin is 31
#JJ4010916        self.setPWM()  #JJ4160616 - Open this one if need to use PWM
        #GPIO.cleanup()

    def setPWM(self):
        global buzzerDutycycle
        self.buzzer_PWM = GPIO.PWM(buzzer_pin, 4000)  #JJ1300516 - Set to 5KHz freq
        self.buzzer_PWM.ChangeDutyCycle(buzzerDutycycle)  #JJ1200616 - Any diff, if we hv this
        #self.buzzer_PWM.start(0)  #JJ1300516 - Need to know how to control this method #JJ1200616

    #def turnOn(self, iteration, duration = 0.25):
    def Beep(self, iteration, duration = 0.25):
        """ JJ1300516 - This method turn on buzzer for the amount of iterations """
        """ JJ1300516 - as specified in its iteration parameter """
        """ JJ3150616 - Parameter: iteration - the number of on off cycle, """
        """ JJ3150616 - duration - period to on the buzzer, default to 25 milli seconds """
        #global buzzer_PWM
        global buzzerDutycycle
        self.buzzer_PWM.ChangeDutyCycle(0)
        self.buzzer_PWM.start(0)  #JJ1300516 - Need to know how to control this method #JJ1200616
        for i in range(0, iteration):
            self.buzzer_PWM.ChangeDutyCycle(buzzerDutycycle)
            time.sleep(duration)
            self.buzzer_PWM.ChangeDutyCycle(0)
            time.sleep(duration)
        self.buzzer_PWM.stop()

    def onBuzzer(self, iteration, duration = 0.20):
        #global buzzerDutycycle
        for i in range(0, iteration):
            self.buzzer_PWM.start(50)
            time.sleep(duration)
            self.buzzer_PWM.start(0)
            time.sleep(duration)
        self.buzzer_PWM.stop()

    #def Beep(self, iteration, duration=0.25):
    def turnOn(self, iteration, duration=0.25):
        #pitch = 1000  #JJ4010916 - 1000 seems to be fine
        pitch = 900
        period = 1.0 / pitch
        delay = period / 2
        cycles = int(duration * pitch)
        for loop in range(iteration):
            for i in range(cycles):
                GPIO.output(buzzer_pin, True)
                time.sleep(delay)
                GPIO.output(buzzer_pin, False)
                time.sleep(delay)
            time.sleep(0.5)
#JJ1300516 - Buzzer Class END --------------------------------------------------


#JJ5030616 - Battery Management Class definition -------------------------------
#JJ2170516 - Battery Manager Class definition
class BatteryManager(object):
    """ JJ2170516 - This class provide API for Battery Gas Gauge LTC2943 device """
    """ set ??? """

    #chargeFlag = 0

    def __init__(self):
        try:
            #I2CBus = smbus.SMBus(1)
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            #JJ2140616 - Set pin 36 as output pin for charge enable;
            GPIO.setup(CHARGE_ENABLE, GPIO.OUT)
            #GPIO.output(CHARGE_ENABLE, False)  #JJ3140916 - ORG
            GPIO.output(CHARGE_ENABLE, True)  #JJ3140916 - Don't enable charge first
            #JJ2140616 - Set pin 38 as input pin for AC presence detection
            GPIO.setup(AC_ADAPTER_PIN, GPIO.IN)
            result = self.getACR()
            if result != -1 and result != 0xBAD:
                #JJ3030816 - Below register can only be set if battery had run thru first charging procedure
                #JJ2140616 - Write configuration value to Config register
                #I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, 0xEC)  #JJ2210616 - Alert Mode
                #I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, 0xEA)  #JJ2210616 - Charge Complete Mode
#JJ3030816                I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, 0xE8)  #JJ2210616 - ALCC disabled
                #0xE9
                #JJ1180716 - Below sttmts used to set threshold to various threshold registers
                #JJ1180716 - Set accumulated charge high and low
                self.setThresholdRegister(CHG_TRHOLD_HIGH_MSB, 0xFFFF)  #JJ1180716 - Addr = 4
                self.setThresholdRegister(CHG_TRHOLD_LOW_MSB, 0xA0F6)  #JJ1180716 - Addr = 6
                #JJ1180716 - Set voltage threshold high and low
                self.setThresholdRegister(V_TRHOLD_HIGH_MSB, 0xB63B)  #JJ1180716 - Addr = 10
                self.setThresholdRegister(V_TRHOLD_LOW_MSB, 0x822B)  #JJ1180716 - Addr = 12
                #JJ1180716 - Set current threshold high and low
                self.setThresholdRegister(CUR_TRHOLD_HIGH_MSB, 0xEEEC)  #JJ1180716 - Addr = 16
                self.setThresholdRegister(CUR_TRHOLD_LOW_MSB, 0x1112)  #JJ1180716 - Addr = 18
                #JJ1180716 - Set temperature threshold high and low
                self.setTemperatureThresholdRegister(0x9F, 0x89)
            else:
                self.setACR(0x7FFF)

            try:
                I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, 0xE8)
            except:
                print("Can't init BatteryManager()")
        except:
            print("Initialise BatteryManager ends with error.....")

    def readFromBattGasGaugeReg(self, address, size):
        """ JJ3180516 - This method read a word value from device at the specified register """
        """ JJ3180516 - This method need to be tested with actual device when they are ready """
        global byteBuffer
        byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        #for j in range(0, 24):
        #    byteBuffer[j] = 0
        tmpAddress = address
        retValue = 0
        try:
            if size == 1:
                byteBuffer[0] = I2CBus.read_byte_data(BATT_ADDR, tmpAddress)
            else:
                #retValue = I2CBus.read_word_data(BATT_ADDR, tmpAddress)
                byteBuffer = I2CBus.read_i2c_block_data(BATT_ADDR, 0, 24)
        except:
            retValue = -1
            print("I2C reads to LTC2943 end with error")
        return retValue

    def getBatteryDeviceStatus(self):
        """ JJ3180516 - This method return the battery device status."""
        """ JJ3180516 - Return value from this method is TBD."""
        #JJ3180516 - This method had been step thru with exception of I2C function;
        #JJ3180516 - so far the logic seems to be correct.
        global byteBuffer
        #global tempMSB
        #global tempLSB
        retStatus = 0
        retStatus = self.readFromBattGasGaugeReg(STATUS_REG, 2)  #JJ3180516 - Official code
        if retStatus == 0:  #JJ1200616 - Return value is good to use
            deviceStatus = byteBuffer[STATUS_REG]
        else:  #JJ1200616 - Return value NG
            deviceStatus = -1
        return deviceStatus

    def getBatteryDeviceControl(self):
        """ JJ3180516 - This method return the battery device control."""
        """ JJ3180516 - Return value from this method is TBD."""
        #JJ3180516 - This method had been step thru with exception of I2C function;
        #JJ3180516 - so far the logic seems to be correct.
        global byteBuffer
        #global tempMSB
        #global tempLSB
        retStatus = 0
        retStatus = self.readFromBattGasGaugeReg(CONTROL_REG, 2)
        if retStatus == 0:  #JJ1200616 - Return value is good to use
            deviceControl = byteBuffer[CONTROL_REG]
        else:  #JJ1200616 - Return value is NG
            deviceControl = -1
        return (deviceControl)

    def configureBatteryManager(self):
        """ JJ2170516 - This method is reserved for codes to configure gas gauge device. """
        """ JJ2170516 - Don't think we need to facilitate RTC configuration. """
        """ JJ2170516 - As such, leave this method blank to preserve for future use. """
        pass

    def getUVLOAlert(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & UVLO_ALERT       #JJ2210616 - 0x01 00000001
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def getVoltageAlert(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & VOLTAGE_ALERT    #JJ2210616 - 0x02 00000010
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def getChargeAlertLow(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & CHRG_ALERT_LOW   #JJ2210616 - 0x04 00000100
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def getChargeAlertHigh(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & CHRG_ALERT_HIGH  #JJ2210616 - 0x08 00001000
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def getTemperatureAlert(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & TEMP_ALERT       #JJ2210616 - 0x10 00010000
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def getACRAlert(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & ACR_ALERT        #JJ2210616 - 0x20 00100000
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def getCurrentAlert(self):
        retStatus = 0
        if self.readFromBattGasGaugeReg(STATUS_REG, 2) == 0:  #JJ2210616 - Return value is good
            devStatus = byteBuffer[STATUS_REG]
            retStatus = devStatus & CURRENT_ALERT    #JJ2210616 - 0x40 01000000
        else:  #JJ2210616 - Return value is bad
            retStatus = -1
        return retStatus

    def setTemperatureThresholdRegister(self, thresholdValueHigh, thresholdValueLow):
        """ JJ1180716 - This method writes threshold value to temperature threshold register """
        #JJ1180716 - JJNOTE: When use this method to set temperature threshold register,
        #JJ1180716 - do take note that these registers are 1 byte and not 2 bytes as compare
        #JJ1180716 - to others threshold registers.
        global outBuffer
        outBuffer[0] = thresholdValueHigh  #JJ1180716 - Temperature threshold high
        outBuffer[1] = thresholdValueLow  #JJ1180716 - Temperature threshold low
        outList = list(outBuffer[0:2])
        retValue = 0
        try:
            I2CBus.write_i2c_block_data(BATT_ADDR, TEMP_TRHOLD_HIGH, outList)  #JJ1180716 - Addr = 22
        except:
            retValue = -1
            print("\setThresholdRegister end with error .....")
        return retValue

    def setThresholdRegister(self, thresholdAddr, thresholdValue):
        """ JJ1200616 - This method writes to threshold register base on thresholdAddr """
        #JJ1200616 - JJNOTE: When use this method to set temperature threshold register,
        #JJ1200616 - do take note that these registers are 1 byte and not 2 bytes as compare
        #JJ1200616 - to others threshold registers.
        global outBuffer
        outBuffer[0] = thresholdValue >> 8  #JJ1200616 - Copy MSB
        outBuffer[1] = thresholdValue & 0x00FF  #JJ1200616 - Copy LSB
        outList = list(outBuffer[0:2])
        retValue = 0
        #"""
        try:
            I2CBus.write_i2c_block_data(BATT_ADDR, thresholdAddr, outList)
        except:
            retValue = -1
            print("\setThresholdRegister end with error .....")
        #"""
        return retValue

    def getThresholdValue(self, thresholdAddr):
        """ JJ1200616 - This method get contents of threshold register based on thresholdAddr """
        #JJ1200616 - JJNOTE: This method is not suitable to get content of temperature
        #JJ1200616 - threshold register as they are 1 byte and not 2 bytes
        global byteBuffer
        if self.readFromBattGasGaugeReg(thresholdAddr, 2) == -1:
            #print("I2C failure .....")
            retThreshold = 0xBAD
        else:
            #JJ1200616 - Here we construct the threshold value
            retThreshold = (byteBuffer[thresholdAddr] << 8) | byteBuffer[thresholdAddr+1]
        return retThreshold

    def setACR(self, ACRValue):
        """ JJ1200616 - This method write to ACR """
        global outBuffer
        outBuffer[0] = ACRValue >> 8  #JJ1200616 - Copy MSB
        outBuffer[1] = ACRValue & 0x00FF  #JJ1200616 - Copy LSB
        outList = list(outBuffer[0:2])
        retValue = 0
        try:
            I2CBus.write_i2c_block_data(BATT_ADDR, ACR_MSB, outList)
        except:
            retValue = -1
            print("\nsetACR end with error .....")
        return retValue

    def getACR(self):
        """ JJ1200616 - This method return the content of Accumulated Charge Register """
        global byteBuffer
        if self.readFromBattGasGaugeReg(ACR_MSB, 2) == -1:
            #print("I2C failure .....")
            retACR = 0xBAD
        else:
            #JJ1200616 - Here we construct the ACR value
            retACR = (byteBuffer[ACR_MSB] << 8) | byteBuffer[ACR_LSB]
            #chrgThresholdHigh = (byteBuffer[CHG_TRHOLD_HIGH_MSB] << 8) | byteBuffer[CHG_TRHOLD_HIGH_LSB]
            chrgThresholdLow = (byteBuffer[CHG_TRHOLD_LOW_MSB] << 8) | byteBuffer[CHG_TRHOLD_LOW_LSB]
            #JJ3030816 - Check if LTC2943 register had been initialised or not
            #JJ3030816 - or for possible lost of register data due to flat battery
            #JJ3030816 - If either condition exist, battery need to go thru first time charging procedure
            #if retACR == 0x7FFE or (chrgThresholdHigh == 0xFFFF and chrgThresholdLow == 0x0000):
            if retACR == 0x7FFE or chrgThresholdLow == 0x0000:
                retACR = -1
                print ("Battery needs to go thru first time charging procedure")
        return retACR

    def getBatteryVoltage(self):
        """ JJ3180516 - This method returns battery voltage to calling method """
        #JJ3180516 - This method had been step thru with exception of I2C function;
        #JJ3180516 - so far the logic seems to be correct.
        #JJ3180516 - Here we set ADC mode to single conversion
        global byteBuffer
        #control, status = self.getBatteryDeviceStatus()
        #JJ4020616 - NOTE: Below remarked code got issue with i2c read; for an un-explained
        #JJ4020616 - reason, every i2c read issued, the device always return value from
        #JJ4020616 - register 0 onward regardless which address we provide.
        #JJ4020616 - If the problem with i2c read is known or resolved, may try to use below
        #JJ4020616 - remarked statements. For now, rely on reading the entire register sets.
        #control = self.getBatteryDeviceControl()  #JJ3180516 - We are interested in control only
        #control = control & 0x07  #JJ4020616 - Clear ADC mode and Prescaler M
        #control = control | 0xE8  #JJ4020616 - Turn on Automatic mode and M = 1024
        #I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, control)  #JJ3180516 - Activate the ADC conversion
        #JJ3180516 - Here we read ADC value from the device
        if self.readFromBattGasGaugeReg(VOLTAGE_MSB, 2) == -1:
            #print("I2C failure .....")
            retVoltage = 0xBAD
        else:
            #JJ4020616 - Here we calculate voltage value
            rawData = (byteBuffer[VOLTAGE_MSB] << 8) | byteBuffer[VOLTAGE_LSB]
            retVoltage = 23.6 * (rawData / 65535.0)  #JJ3180516 - Refer to pg 13
        #print("Inside BatteryManager: "+str('%.2f'%retVoltage))
        return retVoltage

    def getBatteryCurrent(self):
        """ JJ3180516 - This method returns battery current to calling method """
        """ JJ3180516 - This method may not be applicable in our application """
        global byteBuffer
        #status, control = self.getBatteryDeviceStatus()  #JJ3180516 - We are interested in control only
        #control = control & 0x40  #JJ3180516 - Set scan mode to single conversion 01 of bit 6,7
        #I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, control)  #JJ3180516 - Activate the ADC conversion
        #JJ3180516 - Here we read ADC value from the device
        if self.readFromBattGasGaugeReg(CURRENT_MSB, 2) == -1:
            retCurrent = 0xBAD
        else:
            rawData = (byteBuffer[CURRENT_MSB] << 8) | byteBuffer[CURRENT_LSB]
            V_Sense = 60.0
            R_Sense = 40.0  #JJ3180516 - JJNOTE: If H/W change, so will this value
            retCurrent = (V_Sense / R_Sense) * ((rawData - 32767) / 32767.0)  #JJ3180516 - Refer pg 13
        return retCurrent

    def getBatteryTemperature(self):
        """ JJ3180516 - This method returns battery temperature to calling method """
        """ JJ3180516 - This method may not be applicable in our application """
        global byteBuffer
        #status, control = self.getBatteryDeviceStatus()  #JJ3180516 - We are interested in control only
        #control = control & 0x40  #JJ3180516 - Set scan mode to single conversion 01 of bit 6,7
        #I2CBus.write_byte_data(BATT_ADDR, CONTROL_REG, control)  #JJ3180516 - Activate the ADC conversion
        #JJ3180516 - Here we read ADC value from the device
        if self.readFromBattGasGaugeReg(TEMPERATURE_MSB, 2) == -1:
            retTemperature = 0xBAD
        else:
            rawData = (byteBuffer[TEMPERATURE_MSB] << 8) | byteBuffer[TEMPERATURE_LSB]
            #JJ3180516 - Maybe below sttmt may not be needed.
            #rawData = rawData >> 5  #JJ3180516 - Lowest 5 bits of combined register always 0. (pg 14)
            conversionConstant = 273.15
            retTemperature = (510 * (rawData / 65535.0)) - conversionConstant
        return retTemperature  #JJ3180516 -

    #JJ1180716 - This method returns battery level in percentage.
    #JJ1180716 - It return 0, if reading from battery device is not successful
    #JJ3030816 - Return -1, if battery has not been initialised
    def getBatteryLevel(self):
        #JJ1180716 - Need to call respective method to determine any overflow or
        #JJ1180716 - underflow event.
        rawData = self.getACR()
        #print("rawData: " + str(rawData))
        if rawData != 0xBAD:
            battLevel = ((rawData - 0xB2D8) / 0x4C27) * 100
        elif rawData == -1:
            battLevel = -1  #JJ3030816 - Battery not initialised
        else:
            battLevel = 0
        return int(battLevel)

    #JJ3140916 - This method sets pin 36 to low to enable charging process
    def startCharging(self):
        GPIO.output(CHARGE_ENABLE, False)

    #JJ2190716 - This method sets pin 36 to high when battery charging process is done
    def stopCharging(self):
        #GPIO.setup(CHARGE_ENABLE, GPIO.OUT)
        GPIO.output(CHARGE_ENABLE, True)

    #JJ2140616 - This method reads pin 38 (GPIO 20) and return the status of AC adapter
    #JJ2140616 - Return 0 if AC adapter is plugged in, 1 if no AC adapter is plugged
    def chargeStatus(self):
        status = -1
        if GPIO.input(AC_ADAPTER_PIN) == 0:  #JJ3130716 - Pin 38
            status = 1
            #JJ3140916 - Below sttmt may not be necessary
            #GPIO.output(CHARGE_ENABLE, False)  #JJ3140916 - Enable charging
        else:
            status = 0
            #JJ3140916 - Below sttmt may not be necessary
            #GPIO.output(CHARGE_ENABLE, True)  #JJ3140916 - Since AC adapter is not present, disable charging
        return status

#JJ5030616 - Battery Management Class END --------------------------------------


#JJ5100616 - Ambient sensor Class ----------------------------------------------
#JJ4190516 - Ambient sensor Class definition
class AmbientSensor(object):
    """ JJ4190516 This class provide API for ambient sensor device MAX44009 to configure its, """
    """ registers. """

    #JJ5100616 - Our application mainly interested in reading the lux value to determine
    #JJ5100616 - the surrounding's brightness; no interrupt output is necessary to trigger
    #JJ5100616 - anything. As no interrupt is enabled, the upper and lower lux threshold
    #JJ5100616 - registers are not set as well.

    def __init__(self):
        #I2CBus = smbus.SMBus(1)
        #print("Creating Ambient Sensor class.")
        pass

    def readFromAmbientRegister(self, address):
        """ JJ5200516 - This method read a word value from device at the specified register """
        """ JJ5200516 - This method needs to be tested with actual device when it becomes available """
        retValue = 0
        try:
            retValue = I2CBus.read_word_data(AMBIENT_ADDR, address)  #JJ5200516 - OFFICIAL CODE
        except:
            retValue = -1
        #JJ5200516 - Below constitute test data extract from pg 10 of MAX44009 datasheet
        return retValue

    def writeToAmbientRegister(self, startAddr):
        """ JJ5200516 - This method write data to ambient sensor register """
        I2CBus.write_i2c_block_data(AMBIENT_ADDR, startAddr, byteBuffer)

    def getDeviceInterruptStatus(self):
        """ JJ5200516 - This method return the interrupt status of the ambient sensor """
        """ JJ5200516 - 0 - No interrupt occur; 1 - Interrupt kicks in """
        global tempMSB
        global tempLSB
        deviceStatus = self.readFromAmbientRegister(INT_STAT)
        #JJ5200516 - MSB of deviceStatus contains the device interrupt status
        #JJ5200516 - We only interested in this byte
        tempMSB = (deviceStatus >> 8)
        #JJ5200516 - LSB of deviceStatus contains interrupt enable
        tempLSB = (deviceStatus & 0x00FF)
        retValue = (tempMSB & 0x01)  #JJ5200516 - Bit 0 of status register
        return retValue

    def configureAmbient(self):
        """ JJ4190516 - This method is reserved for codes to configure ambient sensor device. """
        """ JJ4190516 - At the moment there is no specific need to change device configuration. """
        """ JJ4190516 - As such, leave this method blank to preserve for future use. """
        retValue = 0
        newConfig = 0x80  #JJ5100616 - This turn on only CONT bit
        try:
            I2CBus.write_byte_data(AMBIENT_ADDR, CFG_REG, newConfig)
        except:
            retValue = -1
        return retValue

    def getConfiguration(self):
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        configReg = 0x00
        try:
            byteBuffer = I2CBus.read_i2c_block_data(AMBIENT_ADDR, CFG_REG, 1)
            configReg = byteBuffer[0]
        except:
            print("\ngetConfiguration end with error .....")
        return configReg

    def getLux(self):
        """ JJ5200516 - This method return lux reading """
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
#        global tempMSB
#        global tempLSB
        luxValue = UPPER_THRESHOLD + 1  #JJ5200516 - To enforce a fault situation
        try:
#            rawData = self.readFromAmbientRegister(LUX_MSB)  #JJ5100616 - Ok using this
            #JJ5200516 - MSB of rawData contains the exponent and MSB of lux mantissa
#            tempMSB = (rawData >> 8)
            #JJ5200516 - LSB of rawData contains LSB of lux mantissa
#            tempLSB = (rawData & 0x00FF)
#            exponent = (tempMSB >> 4)  #JJ5200516 - bit 4 to 7 contains exponent
            byteBuffer = I2CBus.read_i2c_block_data(AMBIENT_ADDR, LUX_MSB, 2)
            exponent = byteBuffer[0] >> 4  #JJ5100616 - Extract exponent
            #JJ5200516 - Pg 9 of MAX44009: if exponent is 15, the reading is overrange
            #JJ5200516 - We need to return a value that represents overrange; at the moment
            #JJ5200516 - remark out the code first, once we know the upper threshold for our
            #JJ5200516 - use case, we just return a value larger then the threshold
            if exponent < 15:
#                mantissa = ((tempMSB & 0x0F) << 4) | (tempLSB & 0x0F)
                mantissa = ((byteBuffer[0] & 0x0F) << 4) | (byteBuffer[1] & 0x0F)
                resolution = 0.045  #JJ5200516 - This needs to change if uses only register 3
                luxValue = (2 ** exponent) * mantissa * resolution  #JJ5200516 - Refer to pg 10
        except:
            luxValue = 0xBAD  #JJ1200616 - Use to indicate i2c read failure
            print("\ngetLux end with error .....")
        #print("\nLux reading: ", luxValue)
        return luxValue

    def getDoorStatus(self):
        """ JJ2140616 - This method return doors status to calling function. """
        """ JJ2140616 - Return 1 if door is open, 0 if door is close. """
        doorStatus = 0
        luxValue = self.getLux()
        if luxValue > DOOR_CLOSE_CONST:
            doorStatus = 1
        return doorStatus

#JJ5100616 - Ambient sensor Class END ------------------------------------------


#JJ5100616 - Power Monitor Class -----------------------------------------------
#JJ3250516 - PowerMonitor Class definition
class PowerMonitor(object):
    """ JJ2070616 This class provide API for current and voltage device INA220 """
    """ to configure its registers."""

    errFlag = 0

    def __init__(self):
        #I2CBus = smbus.SMBus(1)
        #print("Creating PowerMonitor class.")
        self.setConfiguration()  #JJ5170616 - Testing Simple Current Shunt Monitor Usage. pg 15

    #JJ1200616 - This method is not used at the moment
    def readFromPowerMonitorDevice(self, command, size):
        """ JJ3250516 - This method read a byte / word value from device in response to a command """
        """ JJ3250516 - This method needs to be tested with actual device when it becomes available """
        """ JJ3250516 - This method may not be actually used """
        if size == 1:
            retValue = I2CBus.read_byte_data(POW_MONITOR_ADDR, command)  #JJ3250516 - OFFICIAL CODE
        else:
            retValue = I2CBus.read_word_data(POW_MONITOR_ADDR, command)  #JJ3250516 - OFFICIAL CODE
        return retValue

    #JJ1200616 - This method is not used at the moment
    def writeToPowerMonitorDevice(self, command):
        """ JJ3250516 - This method write data to ambient sensor register """
        global byteBuffer
        I2CBus.write_i2c_block_data(POW_MONITOR_ADDR, command, byteBuffer)

    def getConfiguration(self):
        """ JJ3250516 - This method return the value from the configuration register """
        """ JJ3250516 - ??? """
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        #global tempMSB
        #global tempLSB
        byteBuffer[0] = 0  #JJ2020816 - 
        byteBuffer[1] = 0  #JJ2020816 - 
        configVal = 0x00
        byteBuffer = I2CBus.read_i2c_block_data(POW_MONITOR_ADDR, INA_CONFIG_REG, 2)
        configVal = (byteBuffer[0] << 8) | byteBuffer[1]
        return configVal

    def setConfiguration(self):
        """ JJ2070616 - This method set calibration register to a specified setting. """
        #JJ3080616 - JJNOTE: Need to verify the MSB and LSB sequence is sent out correctly
        #JJ3080616 - reading back the register by using getConfiguration method
        global outBuffer
        global errFlag
        configVar = 0x0000  #JJ3080616 - Configuration variable
        #JJ3080616 - Here we configure mode setting to Shunt and Bus continuous
        configVar = INA_MODE3 | INA_MODE2 | INA_MODE1
        #JJ3080616 - Here we configure Shunt ADC resolution to 12 bits
        configVar = configVar | INA_SADC2 | INA_SADC1
        #JJ3080616 - Here we configure Bus ADC resolution to 12 bits
        configVar = configVar | INA_BADC2 | INA_BADC1
        #JJ3080616 - Here we configure PG1 and PG0 for gain at 40mV
        configVar = configVar | GAIN_USED
        #JJ3080616 - Leave BRNG to 0 will set Bus voltage range to 16V
        configVar = configVar | BUS_RANGE_USED
        outBuffer[0] = configVar >> 8  #JJ3080616 - Copy MSB
        outBuffer[1] = configVar & 0x00FF  #JJ3080616 - Copy LSB
        outList = list(outBuffer[0:2])
        try:
            I2CBus.write_i2c_block_data(POW_MONITOR_ADDR, INA_CONFIG_REG, outList)  #JJ3080616 - Official code
        except:
            self.errFlag = 1

    def getShuntVoltage(self):
        """ JJ3080616 - This method read shunt voltage from register 0x01 """
        """ JJ3080616 - The number of sign bits occupy the most significant bits depends on """
        """ JJ3080618 - the PG1 and PG0 (gain) setting. """
        """ JJ3080616 -      0       0  (40mV)  = 4 sign bits """
        """ JJ3080616 -      0       1  (80mV)  = 3 sign bits """
        """ JJ3080616 -      1       0  (160mV) = 2 sign bits """
        """ JJ3080616 -      1       1  (320mV) = 1 sign bit """
        """ JJ3080616 - If contents of register is positive value, take the value and """
        """ JJ3080616 - divide by 100 and return the float value. """
        """ JJ3250516 - This method read raw 2-complements data from temperature register """
        """ JJ3250516 - It contents is negative value, converts the 2-complement into int """
        """ JJ3250516 - value and divide by 100 then return it to calling function. """
        #JJ3080616 - SJNOTE: we may need to get configuratin setting to get gain setting
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        #global tempMSB
        #global tempLSB
        byteBuffer[0] = 0  #JJ2020816 - 
        byteBuffer[1] = 0  #JJ2020816 - 
        negValue = 0
        shuntVoltage = 0
        try:
            byteBuffer = I2CBus.read_i2c_block_data(POW_MONITOR_ADDR, SHUNT_VOLT_REG, 2)
            rawData = (byteBuffer[0] << 8) | byteBuffer[1]
            #JJ4260516 - If rawData is negative, reverse all bits as it is 2-complement value
            if rawData & 0x8000:
                rawData = ~rawData
                rawData = rawData & 0xFFFF  #JJ5170616 - We only interested in 2 bytes
                #JJ3080616 - Then add 1 to complemented value
                rawData = rawData + 1
                negValue = 1

            #JJ3080616 - Get rid of those sign bits accordingly with respect to GAIN setting
            if (GAIN_USED == GAIN_40mV):
                rawData = rawData & 0x0FFF
            elif (GAIN_USED == GAIN_80mV):
                rawData = rawData & 0x1FFF
            elif (GAIN_USED == GAIN_160mV):
                rawData = rawData & 0x3FFF
            else:
                rawData = rawData & 0x7FFF

            #JJ3080616 - Calculate shunt voltage in mVolt
            #intVal = rawData / 100
            #fractVal = (rawData % 100) / 100.0
            #shuntVoltage = intVal + fractVal  #JJ3080616 - Shunt voltage in mV
            shuntVoltage = rawData / 100.0  #JJ3080616 - Shunt voltage in mV
            if negValue == 1:
                shuntVoltage = shuntVoltage * -1.0
        except:
            shuntVoltage = 0xBAD
        return shuntVoltage

    def getBusVoltage(self):
        """ JJ2070616 - This method return bus drop voltage to calling function """
        """ JJ3080616 - Register addr 0x02; content must be shifted right by 3 bits, """
        """ JJ3080616 - the contents of the register is multiplied by 4mV and divide """
        """ JJ3080616 - by 1000 to get unit measurement of volt. """
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        byteBuffer[0] = 0  #JJ2020816 - 
        byteBuffer[1] = 0  #JJ2020816 - 
        busVoltage = 0
        try:
            byteBuffer = I2CBus.read_i2c_block_data(POW_MONITOR_ADDR, BUS_VOLT_REG, 2)
            rawData = (byteBuffer[0] << 8) | byteBuffer[1]
            rawData = rawData >> 3  #JJ3080616 - Shift right 3 bits; refer to pg 22 of INA220
            busVoltage = (rawData * 4) / 1000.0
        except:
            busVoltage = 0xBAD
            print("\ngetBusVoltage end with error .....")
        return busVoltage

    def getPower(self):
        """ JJ2070616 - This method return power data to calling function """
        """ JJ3080616 - Read from register addr 0x03; do take note by reading the contents """
        """ JJ3080616 - of this register will clear CNVR bit in Bus Voltage Register (0x02); """
        """ JJ3080616 - refer to pg 29 of INA220. """
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        byteBuffer[0] = 0  #JJ2020816 - 
        byteBuffer[1] = 0  #JJ2020816 - 
        powerData = 0
        try:
            byteBuffer = I2CBus.read_i2c_block_data(POW_MONITOR_ADDR, POWER_REG, 2)
            powerData = (byteBuffer[0] << 8) | byteBuffer[1]
            #busVoltage = (rawData * 4) / 1000
        except:
            powerData = 0xBAD
            print("\ngetPower end with error .....")
        return powerData

    def getCurrent(self):
        """ JJ2070616 - This method return current to calling function """
        """ JJ3080616 - Read from register addr 0x04. """
        global byteBuffer
        #JJ2230816 - Don't open below sttmt first. Monitor situation
        #byteBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ2230816 - 
        byteBuffer[0] = 0  #JJ2020816 - 
        byteBuffer[1] = 0  #JJ2020816 - 
        negValue = 0
        currentValue = 0
        try:
            byteBuffer = I2CBus.read_i2c_block_data(POW_MONITOR_ADDR, CURRENT_REG, 2)
            currentValue = (byteBuffer[0] << 8) | byteBuffer[1]
            #busVoltage = (rawData * 4) / 1000
            if currentValue & 0x8000:
                currentValue = ~currentValue
                currentValue = currentValue & 0xFFFF  #JJ5170616 - We only interested in 2 bytes
                #JJ5170616 - Then add 1 to complemented value
                currentValue = currentValue + 1
                negValue = 1
            currentValue = currentValue / 1000.0  #JJ517066 - Convert to ampere
            if negValue == 1:
                currentValue = currentValue * -1.0
        except:
            currentValue = 0xBAD
            print("\ngetPower end with error .....")
        return currentValue

    def setCalibration(self, calibrationData=0x0000):
        """ JJ3080616 - This method set calibration data to calibration register, 0x05 """
        #JJ3080616 - JJNOTE: Need to verify the MSB and LSB sequence is sent out correctly
        #JJ3080616 - reading back the register by using getConfiguration method
        global outBuffer
        retValue = 0
        try:
            outBuffer[0] = calibrationData >> 8  #JJ3080616 - Copy MSB
            outBuffer[1] = calibrationData & 0x00FF  #JJ3080616 - Copy LSB
            outList = list(outBuffer[0:2])
            I2CBus.write_i2c_block_data(POW_MONITOR_ADDR, CALIBRATION_REG, outList)  #JJ3080616 - Official code
        except:
            retValue = -1
            print("\nsetCalibration end with error .....")
        return retValue
#JJ5100616 - Power Monitor Class END -------------------------------------------


#JJ2140616 - General (Misc) Class ----------------------------------------------

#JJ2140616 - General Purpose Calss Definition
class GeneralPurpose(object):
    """ JJ2140616 This class provide API for general purpose tasks """

    def __init__(self):
        self.ic = barda.Instrument()
        #self.ic.initialize()  #JJ2260716 - This will also setup GPIO for FPGA_PROG_B, FPGA_INIT_B, and FPGA_DONE
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        #JJ3150616 - Set pin 11 as input pin for FPGA_INIT_B detection
        #GPIO.setup(FPGA_INIT_B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        #JJ3150616 - Set pin 13 as input pin for FPGA_DONE detection
        #GPIO.setup(FPGA_DONE, GPIO.IN)
        #JJ2140616 - Set pin 15 as input pin for POSITION detection
        GPIO.setup(EXIT_SW_PIN, GPIO.IN)
        #JJ2140616 - Set pin 16 as input pin for home switch detection
        GPIO.setup(HOME_SW_PIN, GPIO.IN)
        #JJ2140616 - Set pin 33 as input pin for door switch detection
        GPIO.setup(DOOR_SW_PIN, GPIO.IN)
        #JJ2140616 - Set pin 29 as input pin to detect the present of disposable
        GPIO.setup(DISPOSABLE_SW_PIN, GPIO.IN)
        #JJ2140616 - Set pin 37 as input pin for collision status
        GPIO.setup(COLLISION_PIN, GPIO.IN)
        #JJ2140616 - Set pin 40 as output pin to turn on and off sensor
        GPIO.setup(40, GPIO.OUT)  #JJ2230816 - SENSOR_ENABLE_PIN

    #JJ3150616 - This method can be used to clean / tidy up the system when shutdown.
    #JJ3150616 - At the moment we only do GPIO clean up, but could add those shudown related
    #JJ3150616 - tasks here.
    def deinitialise(self):
        GPIO.cleanup()

    #JJ3150616 - This method reads pin 11 (GPIO 17) and return the status of the pin
    #JJ2140616 - Return 0 if FPGA configuration is ok, 1 if there is error in FPGA configuration
    def detectINIT_B(self):
        status = -1
        status = GPIO.input(FPGA_INIT_B)
        return status

    #JJ2140616 - This method reads pin 13 (GPIO 27) and return the status of the pin
    #JJ2140616 - Return 0 if FPGA configuration is still on, 1 if FPGA configuration is done
    def detectINIT_DONE(self):
        status = -1
        status = GPIO.input(FPGA_DONE)
        return status

    #JJ2140616 - This method reads pin 15 (GPIO 22) and return the status of the pin
    #JJ2140616 - Return 0 if EXIT switch is not detected, 1 if EXIT switch is detected
    def detectExitSwitch(self):
        status = -1
        status = GPIO.input(EXIT_SW_PIN)
        return status

    #JJ2140616 - This method reads pin 16 (GPIO 23) and return the status of home switch
    #JJ2140616 - Return 0 if not at home position, 1 if at home position
    def detectHomeSwitch(self):
        status = -1
        status = GPIO.input(HOME_SW_PIN)
        return status

    #JJ2140616 - This method reads pin 33 (GPIO 13) and return the status of door switch
    #JJ2140616 - Return 0 if door is not closed, 1 if door is closed
    def detectDoorSwitch(self):
        status = -1
        status = GPIO.input(DOOR_SW_PIN)
        return status

    #JJ2140616 - This method read pin 29 (GPIO 5) to determine the present of disposable
    #JJ2140616 - Return 0 if disposable is not loaded, 1 if loaded
    def disposableStatus(self):
        status = -1
        status = GPIO.input(DISPOSABLE_SW_PIN)
        return status

    #JJ2140616 - This method read pin 37 (GPIO 26) to determine the status of collision switch
    #JJ2140616 - Return 0 if there is a collision, 1 if no collision occur
    def collisionStatus(self):
        status = -1
        status = GPIO.input(COLLISION_PIN)
        return status

    #JJ2140616 - This method set pin 40 (GPIO 21) to high to turn on the sensor
    def turnOnSensor(self):
        #GPIO.output(SENSOR_ENABLE_PIN, True)
        self.ic.enable_power(barda.POWER_HOME_LED)

    #JJ2140616 - This method set pin 40 (GPIO 21) to low to turn off the sensor
    def turnOffSensor(self):
        #GPIO.output(SENSOR_ENABLE_PIN, False)
        self.ic.disable_power(barda.POWER_HOME_LED)

#SENSOR_ENABLE_PIN
#JJ2140616 - General (Misc) Class END ------------------------------------------


#MODE1_HOLD = 32
#MODE1_TICK = 35
#MODE1_INT_CTRL = 40

#JJ1220816 - Mode1 Class -------------------------------------------------------
#JJ1220816 - Mode1 class definition
class testMode1(object):
    """ JJ1220816 This class provide API for mode1 test """
    """ JJ2230816 Note that barda.Instrument.initialize() must be run prior to """
    """ JJ2230816 the usage of this class. """

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        #JJ1220816 - Set pin 11 as input pin for FPGA_INIT_B detection
        #GPIO.setup(FPGA_INIT_B, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        #JJ1220816 - Set pin 13 as input pin for FPGA_DONE detection
        #GPIO.setup(FPGA_DONE, GPIO.IN)
        #JJ3240816 - Set pin 26 as output pin for SPIO_CS1 (ADS8320 chip select)
        GPIO.setup(26, GPIO.OUT)  #JJ3240816 - Since barda initialiazation does not do pin 26
        #JJ1220816 - Set pin 32 as output pin for MODE1 hold
        GPIO.setup(MODE1_HOLD, GPIO.OUT)
        #JJ1220816 - Set pin 35 as output pin for MODE1 tick
        GPIO.setup(MODE1_TICK, GPIO.OUT)
        #JJ1220816 - Set pin 40 as output pin for MODE1 integration control
        GPIO.setup(MODE1_INT_CTRL, GPIO.OUT)
        pass

    def testADS8320(self):
#SPI_SS_N - Pin 26
#SPI_SCLK - Pin 23
#SPI_MOSI - Pin 19
#SPI_MISO - Pin 21
        retVolt = 0.0
        GPIO.output(23, True)  #JJ1220816 - SPI_SCLK
        GPIO.output(26, True)  #JJ1220816 - SPI_SS_N
        GPIO.output(19, False) #JJ3240816 - SPI_MOSI

        # Select the device
        GPIO.output(26, False)  #JJ4140716 - Initiates conversion and data transfer
        for i in range(1, 6):  #JJ4140716 - 5 clock periods for conversion cycle
            GPIO.output(23, False)  #JJ1220816 - SPI_SCLK
            pass
            GPIO.output(23, True)  #JJ1220816 - SPI_SCLK
            pass

        #JJ4140716 - Sixth clock period, MISO is enabled, and a low is output
        GPIO.output(23, False)  #JJ1220816 - SPI_SCLK
        pass
        GPIO.output(23, True)  #JJ1220816 - SPI_SCLK
        pass
        dummy = GPIO.input(21)  #JJ1220816 - SPI_MISO

        #JJ4140716 - The next 16 clock periods, the conversion data will be output to MISO
        #tx_byte = tx_string_byte
        rx_byte = 0
        for bitnum in range(15,-1,-1):
            #tx_bit = tx_byte & (1 << bitnum) and True or False
            #GPIO.output(19, tx_bit)  #JJ1220816 - SPI_MOSI
            GPIO.output(23, False)  #JJ1220816 - SPI_SCLK
            pass
            GPIO.output(23, True)  #JJ1220816 - SPI_SCLK
            pass
            rx_byte = (rx_byte << 1) | GPIO.input(21)  #JJ1220816 - SPI_MISO

        # Append the byte to received data
        #rx_data.append(rx_byte)

        #Deselect the device 
        GPIO.output(26, True)  #JJ4140716 - Disable device  #JJ1220816 - SPI_SS_N
        retVolt = (rx_byte / 65535.0) * 5.0
        #retVolt = rx_byte / 3.3  #JJ1220816 - JJTODO: Change 3.3 to a correct value
        #print 'SPI - Rx Data: ' + ''.join('{:02x} '.format(x) for x in rx_data)
        return retVolt

    def getMode1Count(self, integrateTime):
        setValue = True  #JJ1050916 - Replaced below if cond sttmt at control is no longer needed
        """
        if integrateTime >= 0.1 and integrateTime <= 0.5:
            setValue = True
        if integrateTime > 0.5 and integrateTime <= 1.0:
            setValue = False
        """
        #JJ1220816 - Discharge
        GPIO.output(MODE1_TICK, True)
        GPIO.output(MODE1_HOLD, True)
        GPIO.output(MODE1_INT_CTRL, setValue)
        time.sleep(1)
        #JJ1220816 - Integrate
        GPIO.output(MODE1_TICK, False)
        GPIO.output(MODE1_HOLD, False)
        GPIO.output(MODE1_INT_CTRL, setValue)
        time.sleep(integrateTime)
        #JJ1220816 - Read
        GPIO.output(MODE1_TICK, True)
        GPIO.output(MODE1_HOLD, False)
        GPIO.output(MODE1_INT_CTRL, setValue)
        voltRead = self.testADS8320()
        #JJ5260816 - Discharge
        GPIO.output(MODE1_TICK, True)
        GPIO.output(MODE1_HOLD, True)
        GPIO.output(MODE1_INT_CTRL, setValue)
        return voltRead

        
#JJ1220816 - Mode1 Class END ---------------------------------------------------

#JJ5100616 - These are test run method, they are not part of the project -------
def testRunAmbient():
    ambient = AmbientSensor()
    #buzz = Buzzer()
    while True:
        #lux = ambient.getLux()
        door = ambient.getDoorStatus()
        config = ambient.getConfiguration()
        if door == 1:
            print("\nDoor is open")
        else :
            print("\nDoor is close")
        #buzz.turnOn(1)
        time.sleep(5)

#JJ2190716 - This method may be used in production. DO NOT REMOVE AFTER TESTING
def firstTimeCharging():
    global byteBuffer
    global CHG_TRHOLD_HIGH_MSB
    global CHG_TRHOLD_LOW_MSB
    global V_TRHOLD_HIGH_MSB
    global V_TRHOLD_LOW_MSB
    global CUR_TRHOLD_HIGH_MSB
    global CUR_TRHOLD_LOW_MSB
    #JJ2190716 - Local variables used inside the method
    elapsedTime = 0
    chargeFlag = 0
    I_bat = -1.0
    volt = 0
    temperature = 0
    acr = 0
    battery = BatteryManager()

    os.system('clear')
    print("Please ensure AC adapter is turn on.")
    time.sleep(5)
    battery.startCharging()  #JJ3140916 - Start charging process
    while I_bat < 0 and elapsedTime < 60:
        time.sleep(5)
        elapsedTime += 5
        I_bat = battery.getBatteryCurrent()
        if I_bat == 0xBAD:  #JJ3200716 - I2C read error
            print("I2C bus not ready")
            I_bat = -1.0  #JJ3200716 - This prevent it from going into next if sttmt
            break  #JJ3200716 - This is to exit the loop

        if (elapsedTime % 10) == 0:
            print("Please turn on AC adapter.")
            print("Battery current: " + str(I_bat))

    if I_bat > 0:  #JJ2190716 - AC adapter is on now
        print("Battery is now being charged.....")
        print("Hour  Minute  Current  Voltage  Temperature  ACR")
        #print("Hour\tMinute\tCurrent\tVoltage\tTemperature\tACR")
        chargeFlag = 1
        elapsedTime = 0
        minuteTaken = 0
        hourTaken = 0

        while chargeFlag == 1:  #JJ2190716 - Leave the cond as it is now, not sure if we need to have more
            time.sleep(5)
            elapsedTime += 5
            I_bat = battery.getBatteryCurrent()
            #volt = battery.getBatteryVoltage()
            #temperature = battery.getBatteryTemperature()
            #acr = battery.getACR()
            if I_bat == 0xBAD:  #JJ3200716 - I2C read error
                print("I2C bus faulty")
                break;

            if elapsedTime == 60:  #JJ2190716 - Display battery statistic every minute
                elapsedTime = 0  #JJ3200716 - Reset countdown
                minuteTaken += 1
                if minuteTaken == 60:
                    minuteTaken = 0  #JJ3310816 - Reset minute counter
                    hourTaken += 1  #JJ3310816 - Increase hour count by 1

                volt = battery.getBatteryVoltage()
                temperature = battery.getBatteryTemperature()
                #JJ3310816 - Don't use getACR method as it will return -1 when battery is
                #JJ3310816 - fresh. We need to display the content of ACR register during
                #JJ3310816 - charging time.
                if battery.readFromBattGasGaugeReg(ACR_MSB, 2) == -1:
                    #print("I2C failure .....")
                    #retACR = 0xBAD
                    pass
                else:
                    #JJ3310816 - Here we construct the ACR value
                    acr = (byteBuffer[ACR_MSB] << 8) | byteBuffer[ACR_LSB]

                #JJ3310816 - Display statistic of the pass 1 minute of charging
                #print("Hour: "+str(hourTaken)+" Minute: "+str(minuteTaken))
                #print("Current: "+str(I_bat))
                #print("Voltage: "+str(volt))
                #print("Temperature: "+str(temperature))
                #print("ACR: "+str(acr))
                #print(str(hourTaken)+"\t"+str(minuteTaken)+"\t"+str(I_bat)+"\t"+str(volt)+"\t"+str(temperature)+"\t"+str(acr))
                print(str(hourTaken)+"   "+str(minuteTaken)+"   "+str(I_bat)+"   "+str(volt)+"   "+str(temperature)+"   "+str(acr))

            #JJ2190716 - Here we check if battery already full
            if I_bat > 0 and I_bat < 0.05:
                iteration = 0
                while iteration < 3:  #JJ2190716 - Continuous monitor for 3 consecutive drop
                    time.sleep(2)
                    I_bat = battery.getBatteryCurrent()
                    if I_bat < 0.05:
                        iteration += 1
                    else:
                        break

                if iteration > 2:  #JJ2190716 - Finish charging
                    chargeFlag = 0
                    battery.stopCharging()
                    #JJ3030816 - Set accumulated charge register
                    battery.setACR(0xFEFF)
                    print("Set ACR with: "+str(0xFEFF))
                    #JJ3030816 - Set accumulated charge high and low
                    battery.setThresholdRegister(CHG_TRHOLD_HIGH_MSB, 0xFFFF)  #JJ3030816 - Addr = 4
                    battery.setThresholdRegister(CHG_TRHOLD_LOW_MSB, 0xA0F6)  #JJ3030816 - Addr = 6
                    print("Set upper charge threshold with: "+str(0xFFFF))
                    print("Set lower charge threshold with: "+str(0xA0F6))
                    #JJ3030816 - Set voltage threshold high and low
                    battery.setThresholdRegister(V_TRHOLD_HIGH_MSB, 0xB63B)  #JJ3030816 - Addr = 10
                    battery.setThresholdRegister(V_TRHOLD_LOW_MSB, 0x822B)  #JJ3030816 - Addr = 12
                    print("Set upper voltage threshold with: "+str(0xB63B))
                    print("Set lower voltage threshold with: "+str(0x822B))
                    #JJ3030816 - Set current threshold high and low
                    battery.setThresholdRegister(CUR_TRHOLD_HIGH_MSB, 0xEEEC)  #JJ3030816 - Addr = 16
                    battery.setThresholdRegister(CUR_TRHOLD_LOW_MSB, 0x1112)  #JJ3030816 - Addr = 18
                    print("Set upper current threshold with: "+str(0xEEEC))
                    print("Set lower current threshold with: "+str(0x1112))
                    #JJ3030816 - Set temperature threshold high and low
                    battery.setTemperatureThresholdRegister(0x9F, 0x89)
                    print("Set upper temperature threshold with: "+str(0x9F))
                    print("Set lower temperature threshold with: "+str(0x89))
                    print("Battery fully charged.")

            #JJ3200716 - If user switch off AC adapter before charging is complete
            if I_bat < 0:
                chargeFlag = 0
                battery.stopCharging()
                print("AC adapter is switched off before battery finish charging.")
            #time.sleep(5)
    else:  #JJ2190716 - AC adapter is not on after the elapsed time, abort operation
        battery.stopCharging()  #JJ3140916 - Since AC adapter is not turn on, disable charging process
        print("Times up.")
        print("AC adapter has not been turn on.....")

    print("Exit.")


def testRunBatteyManager():
    global CHG_TRHOLD_HIGH_MSB
    global CHG_TRHOLD_LOW_MSB
    global V_TRHOLD_HIGH_MSB
    global V_TRHOLD_LOW_MSB
    global CUR_TRHOLD_HIGH_MSB
    global CUR_TRHOLD_LOW_MSB
    battManager = BatteryManager()
    #buzz = Buzzer()
    status = battManager.getBatteryDeviceControl()
    while True:
        regStatus = battManager.getBatteryDeviceStatus()
        status = battManager.getBatteryDeviceControl()
        print("\n\n\nStatus and Control: " + str(regStatus)+" "+str(status))
        regContent = battManager.getACR()
        if regContent == -1:
            print("Battery has not gone thru first charging procedure yet")
        else:
            print("ACR: " + str(regContent))
        regContent = battManager.getThresholdValue(CHG_TRHOLD_HIGH_MSB)
        print("Charge Thold High: " + str(regContent))
        regContent = battManager.getThresholdValue(CHG_TRHOLD_LOW_MSB)
        print("Charge Thold Low: " + str(regContent))

        regContent = battManager.getBatteryVoltage()
        print("Voltage: " + str(regContent))
        regContent = battManager.getThresholdValue(V_TRHOLD_HIGH_MSB)
        print("Voltage Thold High: " + str(regContent))
        regContent = battManager.getThresholdValue(V_TRHOLD_LOW_MSB)
        print("Voltage Thold Low: " + str(regContent))
        regContent = battManager.getBatteryLevel()
        print("Voltage Level: " + str(regContent))

        regContent = battManager.getBatteryCurrent()
        print("Current: " + str(regContent))
        regContent = battManager.getThresholdValue(CUR_TRHOLD_HIGH_MSB)
        print("Current Thold High: " + str(regContent))
        regContent = battManager.getThresholdValue(CUR_TRHOLD_LOW_MSB)
        print("Current Thold Low: " + str(regContent))

        regContent = battManager.getBatteryTemperature()
        print("Temperature: " + str(regContent))

        #regContent = battManager.chargeStatus()
        #print("Charge: " + str(regContent), "\n")
        #buzz.turnOn(1)
        time.sleep(3)

def testRunGeneralPuspose():
    general = GeneralPurpose()
    while True:
        #collisionStatus = general.collisionStatus()
        #retStatus = general.detectDoorSwitch()
        #general.turnOnSensor()
        #print("\nPin value: ", retStatus)
        #time.sleep(1)
        #general.turnOffSensor()
        #print("\nPin value: ", 0)
        retStatus = general.disposableStatus()
        print("Status: "+str(retStatus))
        time.sleep(1)

def testRunBuzzer():
    buzz = Buzzer()
    #while True:
    for k in range(3):
        buzz.Beep(3)
        time.sleep(1)
    """
    for i in range (0, 3):
        buzz.turnOn(3)
        time.sleep(2)
    for i in range (0, 3):
        buzz.onBuzzer(3)
        time.sleep(2)
    """

def testPowerMonitor():
    pwrMonitor = PowerMonitor()
    #pwrMonitor.setCalibration(0x0DDC)
    while True:
        #tmpConfig = pwrMonitor.getConfiguration()
        shuntV = pwrMonitor.getShuntVoltage()
        busV = pwrMonitor.getBusVoltage()
        #power = pwrMonitor.getPower()
        #amp = pwrMonitor.getCurrent()
        print("Shunt Voltage: "+str(shuntV))
        print("Bus Voltage: "+str(busV))
        #print("Power: "+str(power))
        #print("Current: "+str(amp))
        time.sleep(3)

def testBacklight():
    i = 1
    dutyCycle = 10
    blight = Backlight_PWM()
    #while True:
    while i == 1:
        i = 0
        blight.adjustDutyCycle(100)
        time.sleep(3)
        dutyCycle = 100
        while dutyCycle >= 40:
            dutyCycle = dutyCycle - 5
            blight.adjustDutyCycle(dutyCycle)
            time.sleep(0.03)
        #if dutyCycle < 100:
        #    dutyCycle += 10  #JJ3110516 - Increase duty cycle by 10%
        #else:
        #    dutyCycle = 10  #JJ3110516 - Back to 10%
        #time.sleep(5)
    blight.disableBlightPWM(70)
    #time.sleep(3)

def testRTC():
    rtc = iART_RTC()
    status = rtc.getDeviceStatus()
    if status == 1:  #JJ4250816 - Oscillator has stopped, active it
        rtc.startRTC()
    #rtc.setTime12H(25, 35, 9, 1)
    #dateString = "22.06.2016"
    #timeString = "17.05.20"
#    rtc.setDate("25.08.2016")
#    rtc.setTime("13.45.20")
    SS = 0
    MM = 0
    HH = 0
    NOON = "AM"
    DY = 0
    MN = 0
    YY = 0
    rtc.getDeviceStatus()
    while True:
        #SS, MM, HH, NOON = rtc.getTime()  #retSeconds, retMinutes, retHours ampmString
        #DY, MN, YY = rtc.getDate()  #retDate, retMonth, retYear
        #print("Time is: "+str(HH)+":"+str(MM)+":"+str(SS)+NOON)
        #print("Date is: "+str(DY)+":"+str(MN)+":"+str(2000+YY))
        dateStr = rtc.getDate()
        timeStr = rtc.getTime()
        print("Date: "+dateStr)
        print("Time: "+timeStr)
        time.sleep(5)  #JJ3220616 - After 5 seconds of interval, read again and see it RTC is running

#JJ4140716 - This method is used to test reading using SPI protocol
def setupSPI():
    # Initialize the SPI pins and set their default values
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(24, GPIO.OUT)  #JJ4280416 - SPI_SS_N Pin 24
    GPIO.setup(23, GPIO.OUT)  #JJ4280416 - SPI_SCLK Pin 23
    GPIO.setup(19, GPIO.OUT)  #JJ4280416 - SPI_MOSI Pin 19
    GPIO.setup(21, GPIO.IN)   #JJ4280416 - SPI_MISO Pin 21

    GPIO.output(24, True)
    GPIO.output(23, True)
    GPIO.output(19, False)

#JJ1220816 - This method is used to test Mode1
def testRunMode1():
    setupSPI()
    mode1 = testMode1()
    voltReading = mode1.getMode1Count(0.4)


#JJ5100616 - End of test run method --------------------------------------------

if __name__ == "__main__":
    print("\nMain .....")
    #JJ5030616 - Below sttmt used to test BatteryManager class
    #print("\nEndianess: ", sys.byteorder)  #JJ1130616 - To confirm if machine is Little Endian
    #testRunAmbient()
    firstTimeCharging()
    #testRunBatteyManager()
    #testRunGeneralPuspose()
    #testRunBuzzer()
    #testPowerMonitor()
    #testBacklight()
    #testRTC()
    #testRunMode1()


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



#JJ4050516 ----- End of rtc.py file -----
