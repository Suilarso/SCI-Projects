#/usr/bin/env python

# barda.py
#
# This file implements the BARDA Instrument class which is the API between a Python script and the BARDA custom photon counter.
# This module should not be used as the main instrument control script.  That should be somewhere else and import this module.

from __future__ import division

import re
import struct
import time
import sys

# Try to import the gpio_stub module first.  This is only available on my laptop and so this means I am testing.
# If this module is not available then assume this is the Raspberry PI and import the real GPIO module.  If that one
# doesn't load then the RPi is not configured correctly, so just let the script crash.
#
# Note that the reason the modules are loaded in this order is because on the laptop the script usually runs inside and IDE
# that will cause the script to break on the import exception even when it is caught, which is incredibly annoying.

try:
    import gpio_stub as GPIO
    _real_hardware= False
except ImportError:
    import RPi.GPIO as GPIO
    _real_hardware= True
        
__version__ = '1.0'           # Verion of this module.

# Hardware versions that go with this version of the module.
PCB_VERSION = '1.0'     # Version of PCB that this version of the module works with.
FPGA_VERSION = '1.0'    # Version of FPGA that this version of the module works with.

# GPIO pin usage on the Rev 1.0 PCB.  Note that these are PHYSICAL pin numbers on the header and not Broadcom pins
# FPGA configuration pins
FPGA_PROG_B = 7
FPGA_INIT_B = 11
FPGA_DONE = 13
FPGA_CCLK = 8
FPGA_DIN = 10

# I2C interface
I2C_SDA = 3
I2C_SCL = 5

# SPI interface 
SPI_MOSI = 19
SPI_MISO = 21
SPI_SCLK = 23
SPI_SS_N = 24

# Miscellaneous functions that can't be controlled through SPI
LOGIC_RESET = 18
OSC_STANDBY_N = 22

# FPGA Register Map
REG_FPGA_VERSION = 0x00
REG_PCB_VERSION = 0X01

REG_PHOTON_COUNT = 0x02
REG_GATE_TIMER = 0x06
REG_PHOTON_CTRL_STATUS = 0x0a

REG_STEP_INTERVAL = 0x0b
REG_STEP_COUNT = 0x0f
REG_MOTOR_CTRL_STATUS = 0x13

REG_POWER_CTRL_STATUS = 0x14

# SPI Bus literals
SPI_READ = 0x80
SPI_WRITE = 0x00

FPGA_CLK_RATE = 125000000.0  #JJ4280416 - 125MHz

# Motor parameters
MOTOR_MINIMUM_PERIOD_MS = 1.2

MOTOR_CMD_STOP = 0
MOTOR_CMD_CW = 1
MOTOR_CMD_CCW  = 2
MOTOR_CMD_CW_HOME = 3
MOTOR_CMD_CCW_HOME = 4

MOTOR_STEP_FULL = 0
MOTOR_STEP_HALF = 1
MOTOR_STEP_QUARTER = 2
MOTOR_STEP_MICRO_8 = 3
MOTOR_STEP_MICRO_16 = 4
MOTOR_STEP_MICRO_32 = 5

# Masks for manipulating the motor control register
MOTOR_CMD_BIT_MASK = 0x07
#MOTOR_STEP_SIZE_BIT_MASK = 0x38  #JJ5020916 - Original
MOTOR_STEP_SIZE_BIT_MASK = 0xF8  #JJ5020916 - Replaced
MOTOR_STEP_SIZE_BIT_SHIFT = 3
MOTOR_FAULT_BIT_MASK = 0X40
MOTOR_FAULT_BIT_SHIFT = 6

MOTOR_STEP_DEFAULT = MOTOR_STEP_FULL

POWER_HOME_LED = 0x08
POWER_PMT = 0x04
POWER_MOTOR = 0x02
POWER_BOOST = 0x01
POWER_ALL = (POWER_HOME_LED | POWER_PMT | POWER_MOTOR | POWER_BOOST)

MOTOR_DIR_CW = 0
MOTOR_DIR_CCW = 1

PHOTON_CMD_TRIGGER_COUNTER = 1
PHOTON_STATUS_COUNTING_FLAG_BIT_MASK = 0x01

class Instrument:
    def __init__(self):
        pass
        
    def initialize(self, bitfilename = 'barda_fpga.rbt', force = False):
        # I don't want to hear about it.
        GPIO.setwarnings(False)  #JJ4280416 - Disable warning
        
        # Setup RPi.GPIO to use physical pin numbers
        GPIO.setmode(GPIO.BOARD) 

        # Initialize the FPGA configuration pins
        GPIO.setup(FPGA_PROG_B, GPIO.OUT)  #JJ4280416 - Pin 7
        GPIO.setup(FPGA_INIT_B, GPIO.IN, pull_up_down = GPIO.PUD_UP)  #JJ3150616 - Pin 11; May not be needed here
        GPIO.setup(FPGA_DONE, GPIO.IN)  #JJ3150616 - Pin 13; May not be needed here

        GPIO.setup(FPGA_CCLK, GPIO.OUT)  #JJ4280416 - Pin 8
        GPIO.setup(FPGA_DIN, GPIO.OUT)  #JJ4280416 - Pin 10

        GPIO.output(FPGA_PROG_B, True)
        GPIO.output(FPGA_CCLK, False)
        GPIO.output(FPGA_DIN, False)
        
        # Initialize the SPI pins and set their default values
        GPIO.setup(SPI_SS_N, GPIO.OUT)  #JJ4280416 - Pin 24
        GPIO.setup(SPI_SCLK, GPIO.OUT)  #JJ4280416 - Pin 23
        GPIO.setup(SPI_MOSI, GPIO.OUT)  #JJ4280416 - Pin 19
        GPIO.setup(SPI_MISO, GPIO.IN)   #JJ4280416 - Pin 21
        
        GPIO.output(SPI_SS_N, True)
        GPIO.output(SPI_SCLK, True)
        GPIO.output(SPI_MOSI, False)

        # Configure the miscellaneous logic pins.
        GPIO.setup(LOGIC_RESET, GPIO.OUT)    #JJ4280416 - Pin 18
        GPIO.setup(OSC_STANDBY_N, GPIO.OUT)  #JJ4280416 - Pin 22

        # Turn on the 125MHz oscillator.  We could control it only when needed, but it doesn't draw much current in the overall scheme of
        # things, so we'll turn it on here and leave it be for now.
        self.enable_clk()

        # Configure the FPGA
        self.fpga_configure(bitfilename, force)
    
    def deinitialize(self):
        self.disable_power(POWER_ALL)
        GPIO.cleanup()
        
    def enable_clk(self):
        GPIO.output(OSC_STANDBY_N, True)
        
    def disable_clk(self):
        GPIO.output(OSC_STANDBY_N, False)   
    
    def fpga_reset(self):
        # Clear out the FPGA configuration before a download
        GPIO.output(FPGA_PROG_B, True)
        GPIO.output(FPGA_PROG_B, False)
        GPIO.output(FPGA_PROG_B, True)

    def fpga_configure(self, ascii_bitfile, force = False):
        # Do the configuration download if we are requested by force or if the versions don't match the
        # appropriate ones for this module which most likely means the FPGA isn't programmed.
        
        if _real_hardware and (force == True or not self.verify_version_info()):
            self.fpga_reset()
            
            # Make sure the clock is deasserted, it should be but it never hurts.
            GPIO.output(FPGA_CCLK, False)
            
            # Download the bit stream file.
            binstring_pattern = re.compile(r'[01]*')
            with open(ascii_bitfile, 'r') as f:
                for line in f:
                    line = line.rstrip()
                
                    # See if the line is composed of only ones and zeroes.  If not this is a header line and we will ignore it.
                    binstring_match = re.match(binstring_pattern, line)
                    if line == binstring_match.group(0):
                        for bit in line:
                            if bit == '1':
                                GPIO.output(FPGA_DIN, True)
                            else:
                                GPIO.output(FPGA_DIN, False)
                            GPIO.output(FPGA_CCLK, True)
                            GPIO.output(FPGA_CCLK, False)

        # Reset the logic even if we don't download the configuration image
        self.reset_logic()
            
    def reset_logic(self):
        GPIO.output(LOGIC_RESET, False)
        GPIO.output(LOGIC_RESET, True)
        GPIO.output(LOGIC_RESET, False)

    def spi_transact(self, tx_data):
        # While the Raspberry Pi does have hardware support for SPI, RPi.GPIO does not support
        # it at this time, so we will bit-bang the interface.
        #print 'SPI - Tx Data: ' + ''.join('{:02x} '.format(x) for x in tx_data)
        # Release the bus.  This shouldn't be necessary but just to be sure.
        GPIO.output(SPI_SCLK, True)
        GPIO.output(SPI_SS_N, True)
        
        # Select the device
        GPIO.output(SPI_SS_N, False)

        rx_data = bytearray()
        for tx_string_byte in tx_data:
            # Get the next byte to transmit and reset the receive byte
            tx_byte = tx_string_byte
            rx_byte = 0
            
            # Transmit and receive the bits in this byte
            for bitnum in range(7,-1,-1):
                tx_bit = tx_byte & (1 << bitnum) and True or False
                GPIO.output(SPI_MOSI, tx_bit)
                GPIO.output(SPI_SCLK, False)
                GPIO.output(SPI_SCLK, True)
                rx_byte = (rx_byte << 1) | GPIO.input(SPI_MISO)
                
            # Append the byte to received data
            rx_data.append(rx_byte)
            
        #Deselect the device 
        GPIO.output(SPI_SS_N, True) 
        #print 'SPI - Rx Data: ' + ''.join('{:02x} '.format(x) for x in rx_data)
        return rx_data

    def spi_read(self, address, numbytes):
        readmsg = bytearray(struct.pack('B', address | SPI_READ)) + bytearray(numbytes)
        
        # Return only the data portion, removing the address byte
        return self.spi_transact(readmsg)[1:]
    
    def spi_write(self, address, msgdata):
        # Allow the caller to pass bytearrays or strings and deal with it
        if type(msgdata) is str:
            msgdata = bytearray(msgdata)
            
        # Add the address to the message and send it off
        writemsg = bytearray(struct.pack('B', address | SPI_WRITE)) + msgdata
        self.spi_transact(writemsg)

    def verify_version_info(self):
        (fpga_version, pcb_version) = self.read_version_info()
        if pcb_version != PCB_VERSION or fpga_version != FPGA_VERSION:
            return False
        return True
                
    def read_version_info(self):
        rx_msg = self.spi_read(REG_FPGA_VERSION, 2)
        #(fpga_version_info, pcb_version_info) = struct.unpack('2B', str(rx_msg))  #JJ2120416 - ORG. IDLE Ver 2.7
        (fpga_version_info, pcb_version_info) = struct.unpack('2B', rx_msg)  #JJ2120416 - Replaced. IDLE Ver 3.4
        
        # FPGA Version is a major.minor hardcoded inside the FPGA
        if fpga_version_info == 0xff:
            fpga_version = None
        else:
            fpga_version = '{0}.{1}'.format((fpga_version_info & 0xf0) >> 4, fpga_version_info & 0x0f)
        
        # PCB Version is a major only that is configured by strapping pins on the FPGA to the positive rails with internal pull-downs
        if pcb_version_info == 0xff:
            pcb_version = None
        else:
            pcb_version = '{0}.0'.format(pcb_version_info & 0x0f)
        
        return (fpga_version, pcb_version)
        
    def pack_motor_cmd(self, cmd, step_size):
        regvalue = (cmd & MOTOR_CMD_BIT_MASK) | ((step_size << MOTOR_STEP_SIZE_BIT_SHIFT) & MOTOR_STEP_SIZE_BIT_MASK)
        return regvalue

    def unpack_motor_status(self, regvalue):
        cmd = regvalue & MOTOR_CMD_BIT_MASK
        step_size = (regvalue & MOTOR_STEP_SIZE_BIT_MASK) >> MOTOR_STEP_SIZE_BIT_SHIFT
        fault = (regvalue & MOTOR_FAULT_BIT_MASK) >> MOTOR_FAULT_BIT_SHIFT
        return (cmd , step_size, fault)

    def do_motor_cmd(self, cmd, num_steps, step_period_ms, wait, step_size):
        # Convert the millisecond step interval to the appropriate number of clock ticks
        if step_period_ms < MOTOR_MINIMUM_PERIOD_MS:
            step_period_ms = MOTOR_MINIMUM_PERIOD_MS
        step_interval = FPGA_CLK_RATE * step_period_ms / 1000.0
        step_interval = int(step_interval)  #JJ5290416 - Added to pass self.pack()

        cmd = self.pack_motor_cmd(cmd, step_size)
        motor_msg = struct.pack('>IIB', step_interval , num_steps, cmd)
        
        self.spi_write(REG_STEP_INTERVAL, motor_msg)
        if wait:
            while not self.is_motor_stopped():
                pass
    
    def turn_motor(self, direction, num_steps, step_period_ms, stop_at_home = False, wait = True, step_size = MOTOR_STEP_DEFAULT):
        if direction == MOTOR_DIR_CW and stop_at_home:
            self.do_motor_cmd(MOTOR_CMD_CW_HOME, num_steps, step_period_ms, wait, step_size)
        elif direction == MOTOR_DIR_CW and not stop_at_home:
            self.do_motor_cmd(MOTOR_CMD_CW, num_steps, step_period_ms, wait, step_size)
        elif direction == MOTOR_DIR_CCW and stop_at_home:
            self.do_motor_cmd(MOTOR_CMD_CCW_HOME, num_steps, step_period_ms, wait, step_size)
        elif direction == MOTOR_DIR_CCW and not stop_at_home:
            self.do_motor_cmd(MOTOR_CMD_CCW, num_steps, step_period_ms, wait, step_size)
        else:
             raise ValueError('Invalid motor direction.')
        
    def stop_motor(self):
        cmd = self.pack_motor_cmd(MOTOR_CMD_STOP, MOTOR_STEP_FULL)
        motor_msg = struct.pack('>B', cmd)
        self.spi_write(REG_MOTOR_CTRL_STATUS, motor_msg)
        
    def is_motor_stopped(self):
        status = self.spi_read(REG_MOTOR_CTRL_STATUS, 1)[0]
        (cmd, step_size, fault) = self.unpack_motor_status(status)
        if cmd == MOTOR_CMD_STOP:
            return True
        return False
        
    def enable_power(self, powerbits):
        regvalue = self.spi_read(REG_POWER_CTRL_STATUS, 1)
        regvalue[0] |= powerbits
        self.spi_write(REG_POWER_CTRL_STATUS, regvalue)
    
    def disable_power(self, powerbits):
        regvalue = self.spi_read(REG_POWER_CTRL_STATUS, 1)
        regvalue[0] &= ~powerbits
        self.spi_write(REG_POWER_CTRL_STATUS, regvalue)

    def count_photons(self, gate_time_sec = 1, wait = True):
        # Convert the gate time to clock ticks
        gate_ticks =  gate_time_sec * FPGA_CLK_RATE
        gate_ticks = int(gate_ticks)  #JJ5290416 - Added to pass struct.pack()
        
        photon_msg = struct.pack('>IB', gate_ticks, PHOTON_CMD_TRIGGER_COUNTER)
        self.spi_write(REG_GATE_TIMER, photon_msg)
        if wait:
            while self.is_photon_counter_running():
                pass
            return self.get_photon_count()
        return 0
    
    def is_photon_counter_running(self):
        status = self.spi_read(REG_PHOTON_CTRL_STATUS, 1)[0]
        if status & PHOTON_STATUS_COUNTING_FLAG_BIT_MASK:
            return True
        return False
        
    def get_photon_count(self):
        photon_count_reg = self.spi_read(REG_PHOTON_COUNT, 4)
        #photon_count = struct.unpack('>I', str(photon_count_reg))[0]  #JJ5290416 - ORG
        photon_count = struct.unpack('>I', photon_count_reg)[0]  #JJ5290416 - Replaced
        return photon_count
        
if __name__ == '__main__':
    # Create the instrument controller object and initialize it
    ic = Instrument()
    ic.initialize()
    
#    # Power tests
#    print('Power register at init {0:02X}'.format(ic.spi_read(REG_POWER_CTRL_STATUS, 1)[0]))
#    raw_input('Press enter to continue...')
#    ic.enable_power(POWER_PMT | POWER_BOOST)
#    print('Power register after enable PMT and BOOST {0:02X}'.format(ic.spi_read(REG_POWER_CTRL_STATUS, 1)[0]))
#    raw_input('Press enter to continue...')
#    ic.disable_power(POWER_BOOST)
#    print('Power register after diable BOOST {0:02X}'.format(ic.spi_read(REG_POWER_CTRL_STATUS, 1)[0]))
#    raw_input('Press enter to continue...')
#    ic.enable_power(POWER_PMT | POWER_BOOST)
#    print('Power register after enable PMT and BOOST {0:02X}'.format(ic.spi_read(REG_POWER_CTRL_STATUS, 1)[0]))
#    raw_input('Press enter to continue...')
#    ic.disable_power(POWER_ALL)
#    print('Power register after disable all {0:02X}'.format(ic.spi_read(REG_POWER_CTRL_STATUS, 1)[0]))

#    # Motor Tests
#    ic.enable_power(POWER_HOME_LED | POWER_MOTOR | POWER_BOOST)
#    
#    print('Moving motor clockwise 500 steps at 5 ms/step with Wait = True')
#    ic.turn_motor(MOTOR_DIR_CW, 500, 5)
#    raw_input('Press enter to continue...')
#    print('Moving motor counter clockwise 500 half steps at 5 ms/step with Wait = True')
#    ic.turn_motor(MOTOR_DIR_CCW, 500, 5, step_size = MOTOR_STEP_HALF)
#    raw_input('Press enter to continue...')
#    print('Moving motor clockwise with home detection 2000 steps at 2 ms/step with Wait = True')
#    ic.turn_motor(MOTOR_DIR_CW, 2000, 2, stop_at_home = True)
#    raw_input('Press enter to continue...')
#    print('Moving motor counter clockwise with home detection 2000 steps at 2 ms/step with Wait = False')
#    ic.turn_motor(MOTOR_DIR_CCW, 2000, 2, stop_at_home = True, step_size = MOTOR_STEP_MICRO_32, wait = False)
#    while not ic.is_motor_stopped():
#        time.sleep(1)
#    raw_input('Press enter to continue...')
#    print('Moving motor counter clockwise with home detection 2000 steps at 2 ms/step with Wait = False and forced stop')
#    ic.turn_motor(MOTOR_DIR_CCW, 2000, 2, step_size = MOTOR_STEP_MICRO_32, wait = False)
#    time.sleep(2)
#    ic.stop_motor()
#    raw_input('Press enter to continue...')
#    
#    ic.disable_power(POWER_ALL)
    
#    # Photon counter tests
#    print('Counting photons for 1 second and waiting.')
#    ic.count_photons()
#    print('Counted {0} photons.'.format(ic.get_photon_count()))
#    raw_input('Press enter to continue...')
#    
#    print('Counting photons for 1 second and no waiting.')
#    ic.count_photons(wait = False)
#    while ic.is_photon_counter_running():
#        time.sleep(.25)
#    print('Counted {0} photons.'.format(ic.get_photon_count()))
#    raw_input('Press enter to continue...')
#    
#    print('Counting photons for 2 seconds.')
#    ic.count_photons(2)
#    print('Counted {0} photons.'.format(ic.get_photon_count()))
#    raw_input('Press enter to continue...')
    
    # Crystal on off test
#    (fpga_version, pcb_version) = ic.read_version_info()
#    print('Crystal on - FPGA version {0}, PCB version {1}'.format(fpga_version, pcb_version))
#    ic.disable_clk()
#    (fpga_version, pcb_version) = ic.read_version_info()
#    print('Crystal off - FPGA version {0}, PCB version {1}'.format(fpga_version, pcb_version))
#    ic.enable_clk()
#    (fpga_version, pcb_version) = ic.read_version_info()
#    print('Crystal on - FPGA version {0}, PCB version {1}'.format(fpga_version, pcb_version))
    
    # FPGA check versions test
    (fpga_version, pcb_version) = ic.read_version_info()
    print('FPGA Initialized - FPGA version {0}, PCB version {1}'.format(fpga_version, pcb_version))
#    ic.fpga_reset()
#    (fpga_version, pcb_version) = ic.read_version_info()
#    print('FPGA Configuration cleared - FPGA version {0}, PCB version {1}'.format(fpga_version, pcb_version))
#    ic.deinitialize()
#    ic.initialize()
#    (fpga_version, pcb_version) = ic.read_version_info()
#    print('FPGA Re-initialized - FPGA version {0}, PCB version {1}'.format(fpga_version, pcb_version))
    
    # Logic reset test
    ic.deinitialize()

