#/usr/bin/env python

#*******************************************************************************
#
# FILE     : TestUtility.py
# DATE     : Wed, June 15, 2016
# PROLOGUE : This file implements test utility class to facilitate pre-assembled
#            test on individual hardware component for the component working
#            condition.
# CPU TYPE : ARM Cortec - A53 with BMC2837 core
# PROJECT  : iART
# AUTHOR   : JJ
# VERSION  : ??????
# HISTORY  :
#   JJ3150616 - Start coding:
#   JJ3150616 - Create MainMenu class definition
#   JJ3150616 - Add method to display menu item for selection
#   JJ3150616 - Add method to test buzzer; buzzerTest()
#   JJ3150616 - Add method to test temperature; temperatureTest()
#   JJ3150616 - Add method to test ambient; ambientTest()
#   JJ4160616 - Add method to test battery operational voltage; batteryTest()
#   JJ4160616 - Add method to test motor movement; motorTest()
#   JJ1130616 - Test run the code on TI evaluation board
#   JJ2160816 - Add methods to test various sensor functionality; homeSwitch(), exitSwitch()
#   JJ2160816 - disposableDetection(), doorDetection(), collisionSwitch()
#
#*******************************************************************************

# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

#from __future__ import division

#import re
#import struct
import os  #JJ3220616 - Need it to help clear screen
import time
import sys
import InHouseAPI
import Thermostat75C
import barda
#import smbus     #JJ4180516 - For I2C protocol
#import usb.core  #JJ2190416 - Testing USB
#import usb.util  #JJ2190416 -
import RPi.GPIO as GPIO  #JJ2260416 - Import GPIO class
from log import log  #JJ5111116 - For writing battery result to log file


# I2C interface
#I2C_SDA = 3
#I2C_SCL = 5
# SPI interface 

# FPGA Register Map
#JJ4050516 REG_FPGA_VERSION = 0x00
#JJ4050516 REG_PCB_VERSION = 0X01

# SPI Bus literals


#FPGA_CLK_RATE = 125000000.0

# Motor parameters

# Masks for manipulating the motor control register

#JJ3150616 - Create a class by the name of MainMenu
#JJ3150616 - Methods will be as follow: 

#JJ2310516 - Both DS1631 and TMP75 use the same slave address structure
#THERMO_ADDR = 0x48   #JJ3250516 - DS1631 Thermometer & Thermostat Device (A0, A1, A2 connect to ground)
#RTC_ADDR_WR = 0xD0 #JJ4050516 - 11010000
#RTC_ADDR_RD = 0xD1 #JJ4050516 - 11010001

#I2CBus = smbus.SMBus(1)

#JJ3010616 - Below is TMP75 Register (Command) Set
#TEMPERATURE_REG = 0x00  #JJ3010616 - Command to read converted temperature
#CONFIG_REG = 0x01  #JJ3010616 - Command to configure TMP75 operation
#T_LOW_REG = 0x02  #JJ3010616 - Command to set threshold low register
#T_HIGH_REG = 0x03  #JJ3010616 - Command to set threshold high register
#ONE_SHOT_REG = 0x04  #JJ4090616 - Write anything to activate one-shot conversion

#JJ3010616 - Below is configuration register bit format
#SD_BIT = 0x0100
#TM_BIT = 0x0200
#POL_BIT = 0x0400
#F0_BIT = 0x0800
#F1_BIT = 0x1000
#OS_BIT = 0x2000


#DEFAULT_CONFIG = F1_BIT | F0_BIT  #JJ3010616 - equiv 0x1800
#UPPER_THRESHOLD = 0x4B00  #JJ4090616 - Set to 75; subject to change
#LOWER_THRESHOLD = 0xE700  #JJ4090616 - Set to -25; subject to change


#JJ3150616 - Main menu table
menuItem = ["1) Speaker",
            "2) Temperature",
            "3) Ambient Sensor",
            "4) Battery",
            "5) Power Monitor",
            "6) Motor - Auto",
            "7) Motor - Manual",
            "8) Home Switch",
            "9) Exit Switch",
            "10) Disposalbe Detection",
            "11) Door Detection",
            "12) Collision Switch",
            "13) ADC Voltage",
            "14) Motor - Without sensor",
            "15) Simulate Rough Calibration",
            "16) Simulate Fine Calibration",
            "17) Quit"]
maxMenuItem = 17  #JJ3150616 - This number must match the number of items in the menu
menuPrompt = "Key in '1' to '"+str(maxMenuItem)+"' for your selection: "

byteBuffer = bytearray([0,0,0,0,0,0,0,0]) #JJ4190516 - I/O byte buffer
#outBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])  #JJ4190516 - Output buffer to write to EEPrompt
#inBuffer = bytearray([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])   #JJ4190516 - Input buffer to contain page read from EEPrompt

#tempMSB = 0
#tempLSB = 0

firstTimeCounter = 1
moveDirection = 1  #JJ1290816 - 1 - Toward exit, 2 - Toward home

#JJ3150616 - MainMenu Class definition
class MainMenu(object):
    """ JJ3150616 This class create test utility main menu and provide method """
    """ JJ3150616 to carry out the test """

    def __init__(self):
        #global 
        #I2CBus = smbus.SMBus(1)
        self.m_log = log()  #JJ5111116 - Creating log gile
        print("Creating MainMenu class.")

    def displayMenu(self):
        """ JJ3150616 - This method displays menu items to user for selection """
        """ JJ3150616 -  """
        """ JJ3150616 -  """
        os.system('clear')
        for ndx in range(0, maxMenuItem):
            print(menuItem[ndx])

    def getMenuInput(self):
        """ JJ3250516 - This method prompts user for input from menu selection """
        #global byteBuffer
        selection = 0
        selection = input(menuPrompt)
        return selection

    def buzzerTest(self):
        """ JJ3250516 - This method tests buzzer """
        global firstTimeCounter
        firstTimeCounter = 1
        buzzer = InHouseAPI.Buzzer()
        for i in range (0, 3):
            print("Buzz "+str(i+1)+" : ")
            buzzer.turnOn(1)
            time.sleep(0.25)

    def temperatureTest(self):
        """ JJ3150616 - This method tests the working condition of temperature device """
        global firstTimeCounter
        firstTimeCounter = 1
        thermostat = Thermostat75C.Thermostat()
        for i in range (0, 3):
            temp = thermostat.getTemperature()
            print("Reading "+str(i+1)+" is: ", temp)
            time.sleep(0.5)

    def ambientTest(self):
        """ JJ3250516 - This method tests ambient sensor working condition """
        """ JJ3250516 - ??? """
        global firstTimeCounter
        firstTimeCounter = 1
        ambient = InHouseAPI.AmbientSensor()
        for i in range (0, 3):
            lux = ambient.getLux()
            print("Reading "+str(i+1)+" is: ", lux)
            time.sleep(0.5)

    def batteryTest(self):
        """ JJ4160616 - This method tests battery current operational voltage. """
        """ JJ4160616 -  """
        """ JJ4160616 -  """
        global firstTimeCounter
        firstTimeCounter = 1
        battManager = InHouseAPI.BatteryManager()
        #time.sleep(0.50)  #JJ4160616 - 100 milliseconds delay
        print("Current battery statistics are as follow:")
        while True:
            acr = battManager.getACR()  #JJ1050916 -
            volt = battManager.getBatteryVoltage()
            current = battManager.getBatteryCurrent()
            temperature = battManager.getBatteryTemperature()
            charge = battManager.chargeStatus()
            battLvl = battManager.getBatteryLevel()  #JJ3161116 - 
            #JJ1050916 - Print ACR value
            if acr == 0xBAD or acr == -1:
                print("Error ...")
            else:
                self.m_log.WriteLogErr("ACR: "+str(acr))
                #print("ACR: "+str(acr))
            #JJ5170616 - Print voltage
            if volt == 0xBAD:
                print("Error ...")
            else:
                self.m_log.WriteLogErr("ACR: "+str('%.2f' % volt))
                #print("Voltage: "+str('%.2f' % volt))
            #JJ5170616 - Print current
            if current == 0xBAD:
                print("Current: Error")
            else:
                self.m_log.WriteLogErr("ACR: "+str('%.9f' %  current))
                #print("Current: "+str('%.9f' %  current))
            #JJ5170616 - Print temperature
            if temperature == 0xBAD:
                print("Temperature: Error")
            else:
                self.m_log.WriteLogErr("ACR: "+str('%.2f' %  temperature))
                #print("Temperature: "+str('%.2f' %  temperature))

            self.m_log.WriteLogErr("Level: "+str(battLvl))  #JJ3161116 - Battery level

            #JJ5170616 - Print charge status
            if charge == 0xBAD:
                print("Charge: Error")
            else:
                print("Charge: "+str(charge))

            print("Battery test complete.")
            time.sleep(30)

    def powerTest(self):
        """ JJ5170616 - This method tests power and current monitor """
        global firstTimeCounter
        firstTimeCounter = 1
        pwrMonitor = InHouseAPI.PowerMonitor()
        #pwrMonitor.setCalibration(0x0DDC)
        for i in range (0, 3):
            shuntV = pwrMonitor.getShuntVoltage()
            busV = pwrMonitor.getBusVoltage()
            #power = pwrMonitor.getPower()
            #amp = pwrMonitor.getCurrent()
            print("Shunt Voltage reading "+str(i+1)+" : "+str(shuntV))
            print("Bus Voltage reading "+str(i+1)+" : "+str(busV))
            #print("Power "+str(i+1)+" : "+str(power))
            #print("Current "+str(i+1)+" : "+str(amp))
            time.sleep(0.5)

    def getTemperature(self):
        """ JJ3250516 - This method read raw 2-complements data from temperature register """
        """ JJ3250516 - It converts the 2-complement into float value and return it to calling """
        """ JJ3250516 - function """
        #global tempMSB
        #global tempLSB
        #global byteBuffer
        try:
            pass
        except:
            print("\ngetTemperature end with error .....")

        #print("\nTemperature is: ", temperature)
        pass

    def motorTest(self):
        """ JJ4160616 - This method tests motor movements. """
        global firstTimeCounter
        firstTimeCounter = 1
        steps = 7000
        pulse = 1.0  #[0.8, 1.0, 2.0]
        GP.turnOnSensor()
        print("Enable Auto motor test.")
        ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        #JJ1290816 - This is to bring the tray to home position
        print("Moving to Home postion")
        ic.turn_motor(barda.MOTOR_DIR_CW, 100, pulse, stop_at_home = False, step_size = barda.MOTOR_STEP_HALF)
        #ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = True, wait = False, step_size = barda.MOTOR_STEP_HALF)  #JJ4131016 - Using FPGA
        #"""
        ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = True, wait = False, step_size = barda.MOTOR_STEP_HALF)  #JJ4131016 - Using Home sensor
        while GP.detectHomeSwitch() == 0:
            pass
        ic.stop_motor()
        #"""
        print("At Home postion now")
        for i in range (0, 50):
            print("Iteration: "+str(i+1)+" moving toward exit position")
            ic.turn_motor(barda.MOTOR_DIR_CW, steps, pulse, stop_at_home = False, wait = False, step_size = barda.MOTOR_STEP_HALF)
            while GP.detectExitSwitch() == 0:
                pass
            ic.stop_motor()
            print("Iteration: "+str(i+1)+" moving toward home position")
            #ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = True, wait = False, step_size = barda.MOTOR_STEP_HALF)  #JJ4131016 - Using FPGA
            #"""
            ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = True, wait = False, step_size = barda.MOTOR_STEP_HALF)  #JJ4131016 - Using Home sensor
            while GP.detectHomeSwitch() == 0:
                pass
            ic.stop_motor()
            #"""
        GP.turnOffSensor()
        ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        print("Disable motor power.")

    def motorManualTest(self):
        """ JJ4160616 - This method tests motor movements. """
        global firstTimeCounter
        global moveDirection
        steps = 6470
        pulse = 1.5  #[0.8, 1.0, 2.0]
        GP.turnOnSensor()
        ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        #JJ1290816 - This is to bring the tray to home position
        if firstTimeCounter == 1:
            print("Enable Manual motor test.")
            print("Moving to Home postion")
            ic.turn_motor(barda.MOTOR_DIR_CW, 100, pulse, stop_at_home = False, step_size = barda.MOTOR_STEP_HALF)
            ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = True, wait = True, step_size = barda.MOTOR_STEP_HALF)  #JJ4131016 - Using FPGA
            """
            ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = False, wait = False, step_size = barda.MOTOR_STEP_HALF)  #JJ4131016 - Using Home Sensor
            while GP.detectHomeSwitch() == 0:
                pass
            ic.stop_motor()
            """
            print("At Home postion now")
            firstTimeCounter = 0
            moveDirection = 1

        if moveDirection == 1:
            print("Moving toward exit position")
            ic.turn_motor(barda.MOTOR_DIR_CW, steps, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
            moveDirection = 2
        elif moveDirection == 2:
            print("Moving toward home postition")
            ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
            moveDirection = 1

        GP.turnOffSensor()
        ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        print("Disable motor power.")

    def homeSwitch(self, GP):
        global firstTimeCounter
        firstTimeCounter = 1
        print("Home Switch")
        GP.turnOnSensor()
        for i in range (0, 5):
            pinStatus = GP.detectHomeSwitch()
            print("Pin status: ", pinStatus)
            time.sleep(0.5)
        GP.turnOffSensor()

    def exitSwitch(self, GP):
        global firstTimeCounter
        firstTimeCounter = 1
        print("Exit Switch")
        GP.turnOnSensor()
        for i in range (0, 5):
            pinStatus = GP.detectExitSwitch()
            print("Pin status: ", pinStatus)
            time.sleep(0.5)
        GP.turnOffSensor()

    def disposableDetection(self, GP):
        global firstTimeCounter
        firstTimeCounter = 1
        print("Disposable Detection")
        GP.turnOnSensor()
        for i in range (0, 5):
            pinStatus = GP.disposableStatus()
            print("Pin status: ", pinStatus)
            time.sleep(0.5)
        GP.turnOffSensor()

    def doorDetection(self, GP):
        global firstTimeCounter
        firstTimeCounter = 1
        print("Door Detection")
        GP.turnOnSensor()
        for i in range (0, 5):
            pinStatus = GP.detectDoorSwitch()
            print("Pin status: ", pinStatus)
            time.sleep(0.5)
        GP.turnOffSensor()

    def collisionSwitch(self, GP):
        global firstTimeCounter
        firstTimeCounter = 1
        print("Collision Switch")
        #GP.turnOnSensor()
        for i in range (0, 5):
            pinStatus = GP.collisionStatus()
            print("Pin status: ", pinStatus)
            time.sleep(0.5)
        #GP.turnOffSensor()

    def testMode1(self):
        global firstTimeCounter
        firstTimeCounter = 1
        ic.enable_power(barda.POWER_PMT)
        # Initialize the SPI pins and set their default values
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(26, GPIO.OUT)  #JJ4280416 - SPI_SS_N Pin 26
        GPIO.setup(23, GPIO.OUT)  #JJ4280416 - SPI_SCLK Pin 23
        GPIO.setup(19, GPIO.OUT)  #JJ4280416 - SPI_MOSI Pin 19
        GPIO.setup(21, GPIO.IN)   #JJ4280416 - SPI_MISO Pin 21

        GPIO.output(26, True)
        GPIO.output(23, True)
        GPIO.output(19, False)

        mode1 = InHouseAPI.testMode1()
        for i in range (0, 5):
            voltReading = mode1.getMode1Count(0.1)
            print("Volt: ", voltReading)
            time.sleep(1)
        ic.disable_power(barda.POWER_PMT)

    def motorNoSensor(self):
        """ JJ4010916 - This method tests motor movements without utilising """
        """ JJ4010916 - sensor to stop its movements. """
        global firstTimeCounter
        firstTimeCounter = 1
        rounds = 5
        steps = 5182  #JJ4010916 - This amount of steps will cover about 75% of the entire track
        pulse = 1.5
        print("Enable Motor test with no sensors.")
        ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        for i in range (0, rounds):
            print("Test run "+str(i+1)+" of "+str(rounds)+".....")
            print("Moving clockwise "+str(steps)+" steps")
            ic.turn_motor(barda.MOTOR_DIR_CW, steps, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
            print("Moving counter clockwise "+str(steps)+" steps")
            ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
        ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        print("Disable motor power.")

    def simulateRoughCalibration(self):
        """ JJ4160616 - This method tests motor movements. """
        global firstTimeCounter
        firstTimeCounter = 1
        steps = 20
        pulse = 1.0  #[0.8, 1.0, 2.0]
        GP.turnOnSensor()
        print("Enable Simulation rough calibration test.")
        ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        #JJ1290816 - This is to bring the tray to home position
        #print("Moving to Home postion")
        #ic.turn_motor(barda.MOTOR_DIR_CW, 100, pulse, stop_at_home = False, step_size = barda.MOTOR_STEP_HALF)
        #ic.turn_motor(barda.MOTOR_DIR_CCW, 1170, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
        #while GP.detectHomeSwitch() == 0:
        #    pass
        #ic.stop_motor()
        #print("At Home postion now")
        for i in range (0, 100):
            print("Iteration: "+str(i+1))
            #ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
            #time.sleep(0.10)
            ic.turn_motor(barda.MOTOR_DIR_CW, steps, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
            #ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
            time.sleep(0.10)

        print("Rewinding motor")
        #ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        #time.sleep(0.1)
        ic.turn_motor(barda.MOTOR_DIR_CCW, 2000, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
        #ic.stop_motor()
        GP.turnOffSensor()
        ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        print("Disable motor power.")

    def simulateFineCalibration(self):
        """ JJ4160616 - This method tests motor movements. """
        global firstTimeCounter
        firstTimeCounter = 1
        steps = 5
        pulse = 1.0  #[0.8, 1.0, 2.0]
        GP.turnOnSensor()
        print("Enable Simulation fine calibration test.")
        ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        #JJ1290816 - This is to bring the tray to home position
        #print("Moving to Home postion")
        #ic.turn_motor(barda.MOTOR_DIR_CW, 100, pulse, stop_at_home = False, step_size = barda.MOTOR_STEP_HALF)
        #ic.turn_motor(barda.MOTOR_DIR_CCW, 1170, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
        #while GP.detectHomeSwitch() == 0:
        #    pass
        #ic.stop_motor()
        #print("At Home postion now")
        for i in range (0, 407):
            print("Iteration: "+str(i+1))
            #ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
            #time.sleep(0.10)
            ic.turn_motor(barda.MOTOR_DIR_CW, steps, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
            #ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
            time.sleep(0.10)
            #while GP.detectExitSwitch() == 0:
            #    pass
            #ic.stop_motor()
            #print("Iteration: "+str(i+1)+" moving toward home position")
            #ic.turn_motor(barda.MOTOR_DIR_CCW, steps, pulse, stop_at_home = False, wait = False, step_size = barda.MOTOR_STEP_HALF)
            #while GP.detectHomeSwitch() == 0:
            #    pass

        print("Rewinding motor")
        #ic.enable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        ic.turn_motor(barda.MOTOR_DIR_CCW, 2035, pulse, stop_at_home = False, wait = True, step_size = barda.MOTOR_STEP_HALF)
        ic.stop_motor()
        GP.turnOffSensor()
        ic.disable_power(barda.POWER_MOTOR | barda.POWER_BOOST)
        print("Disable motor power.")

if __name__ == "__main__":
    global firstTimeCounter
    GP = InHouseAPI.GeneralPurpose()
    ic = barda.Instrument()
    ic.initialize()
    #ic.enable_power(barda.POWER_PMT)
    HWTest = MainMenu()
    while True:
        HWTest.displayMenu()
        selection = int(HWTest.getMenuInput())
        if selection == 1:
            print("Testing Buzzer")
            HWTest.buzzerTest()
        elif selection == 2:
            print("Testing Temperature")
            HWTest.temperatureTest()
        elif selection == 3:
            print("Testing Ambient Sensor")
            HWTest.ambientTest()
        elif selection == 4:
            print("Testing Battery")
            HWTest.batteryTest()
        elif selection == 5:
            print("Testing Power Monitor")
            HWTest.powerTest()
        elif selection == 6:
            print("Testing Motor")
            HWTest.motorTest()
        elif selection == 7:
            print("Testing Motor")
            HWTest.motorManualTest()
        elif selection == 8:
            print("Home Switch")
            HWTest.homeSwitch(GP)
        elif selection == 9:
            print("Exit Switch")
            HWTest.exitSwitch(GP)
        elif selection == 10:
            print("Disposable Detection")
            HWTest.disposableDetection(GP)
        elif selection == 11:
            print("Door Detection")
            HWTest.doorDetection(GP)
        elif selection == 12:
            print("Collision Switch")
            HWTest.collisionSwitch(GP)
        elif selection == 13:
            print("ADC Voltage")
            HWTest.testMode1()
        elif selection == 14:
            print("Motor test with no sensors")
            HWTest.motorNoSensor()
        elif selection == 15:
            print("Simulate Rough Calibration")
            HWTest.simulateRoughCalibration()
        elif selection == 16:
            print("Simulate Fine Calibration")
            HWTest.simulateFineCalibration()
        elif selection == maxMenuItem:
            GP.deinitialise()
            #ic.disable_power(barda.POWER_ALL)
            print("Good bye .....")
            break
        else:
            print("Invalid selection")

    print("Quiting Test Utility")
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
