import os
import threading
import time
import RPi.GPIO as GPIO

"""
def isr(path, interrupt):
    prev_mod = os.stat(path).st_mtime
    while(1):  
        new_mod = os.stat(path).st_mtime
        if new_mod != prev_mod:
            print ("Updates! Waiting to begin")
            # Prevent enter into critical code and updating
            # While the critical code is running.
            with interrupt:     
                print ("Starting updates")
                prev_mod = new_mod
                print ("Fished updating")
        else:
            print ("No updates")
            time.sleep(1)


def func2(interrupt): 
    while(1):
        with interrupt:     # Prevent updates while running critical code
            # Execute critical code
            print ("Running Crit Code")
            time.sleep(5)
            print ("Finished Crit Code")
        # Do other things
"""

#interrupt = threading.Lock()

#path = "testfil.txt"
#t1 = threading.Thread(target = isr, args = (path, interrupt))
#t2 = threading.Thread(target = func2, args = (interrupt,))
#t1.start()
#t2.start()

# Create and "Update" to the file
#time.sleep(12)
#chngfile = open("testfil.txt","w")
#chngfile.write("changing the file")
#chngfile.close()
#time.sleep(10)

#threading.Thread  #JJ2111016 - Inherit
#frequency=1250, dutyCycle=100
class isrBell(threading.Thread):
    def __init__(self, Freq, Dutycycle):
        self.frequency = Freq  #1250
        self.dutyCycle = Dutycycle #50
        super(isrBell, self).__init__()
        #super(isrBell, self)
        #Thread.__init__(self)

    def run(self):
        global frequency
        global dutyCycle
        onPeriod = 0
        offPeriod = 0
        milliSec = frequency / 1000
        buzzer_pin = 31
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(buzzer_pin, GPIO.OUT)  #JJ2111016 - buzzer_pin is 31
        onPeriod = ((milliSec * dutyCycle) / 100) / 1000
        offPeriod = milliSec - onPeriod  #100 - dutyCycle
        while(1):
            GPIO.output(buzzer_pin, True)
            time.sleep(onPeriod)
            GPIO.output(buzzer_pin, False)
            time.sleep(offPeriod)

#bell = threading.Thread(target=isrBell, args=(1250, 50))
bell = isrBell(1250, 50)
bell.start()
time.sleep(3)
bell.exit()


