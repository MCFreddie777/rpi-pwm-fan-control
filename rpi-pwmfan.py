#!/usr/bin/python3

import wiringpi as wiringpi
import time
from time import sleep

PWM_PIN = 12  # HW PWM works on GPIO 12, 13, 18 and 19 on RPi4B
RPM_MAX = 5000  # Noctua Specs: Max=5000
RPM_MIN = 1500  # Noctua Specs: Min=1000
TACHO_PIN = 6
MAX_TEMP = 60  # Above this temperature, the FAN is at max speed
LOW_TEMP = 55  # Lowest temperature, if lowest of this, the FAN is Off
WAIT = 2  # Interval before adjusting RPM (seconds)

rpmChkStartTime = None
rpmPulse = 0

###PID Parameters###
KP = 2
KI = 1
KD = 1
TAU = 1
PID_MIN = 0
PID_MAX = 100


class PID_Controller:
    def __init__(self, kp, ki, kd, tau, limMin, limMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tau = tau
        self.limMin = limMin
        self.limMax = limMax
        self.time = WAIT
        self.integrator = 0
        self.prevError = 0
        self.differentiator = 0
        self.prevMeasure = 0
        self.out = 0
    
    def update(self, setpoint, measure):
        error = setpoint - measure
        # error=measure-setpoint
        # Proportional gain
        proportional = self.kp * error
        # Integral gain
        self.integrator = self.integrator + 0.5 * self.ki * self.time * (error + self.prevError)
        # Anti-wind-up
        if self.limMax > proportional:
            intLimMax = self.limMax - proportional
        else:
            intLimMax = 0
        if self.limMin < proportional:
            intLimMin = self.limMin - proportional
        else:
            intLimMin = 0
        # Clamp integrator
        if self.integrator > intLimMax:
            self.integrator = intLimMax
        else:
            self.integrator = intLimMin
        # Differentiator gain
        self.differentiator = (2 * self.kd * measure - self.prevMeasure) + (
                2 * self.tau - self.time) * self.differentiator / (2 * self.tau + self.time)
        # Calculate output
        self.out = proportional + self.integrator + self.differentiator
        # Apply limits
        if self.out > self.limMax:
            self.out = self.limMax
        elif self.out < self.limMin:
            self.out = self.limMin
        # Store data
        print(self.prevError)
        self.prevError = error
        print(self.prevError)
        self.prevMeasure = measure


myPID = PID_Controller(KP, KI, KD, TAU, PID_MIN, PID_MAX)


def getCPUTemp():
    f = open('/sys/class/thermal/thermal_zone0/temp', 'r')
    temp = f.readline()
    f.close()
    ret = float(temp) / 1000
    return ret


def tachoISR():
    global rpmPulse
    rpmPulse += 1
    return


def setupTacho():
    global rpmChkStartTime
    
    print("Setting up Tacho input pin")
    wiringpi.wiringPiSetupGpio()
    wiringpi.pinMode(TACHO_PIN, wiringpi.INPUT)
    wiringpi.pullUpDnControl(TACHO_PIN, wiringpi.PUD_UP)
    rpmChkStartTime = time.time()
    wiringpi.wiringPiISR(TACHO_PIN, wiringpi.INT_EDGE_FALLING, tachoISR)
    return


def readRPM():
    global rpmPulse, rpmChkStartTime
    fanPulses = 2
    
    duration = time.time() - rpmChkStartTime
    frequency = rpmPulse / duration
    ret = int(frequency * 60 / fanPulses)
    rpmChkStartTime = time.time()
    rpmPulse = 0
    print("Frequency {:3.2f} | RPM:{:4d}".format(frequency, ret))
    return ret


def fanOn():
    wiringpi.pwmWrite(PWM_PIN, RPM_MAX)
    return


def updateFanSpeed():
    temp = getCPUTemp()
    myPID.update(LOW_TEMP, temp)
    
    if myPID.out < 0:
        percentDiff = 0
    else:
        percentDiff = myPID.out
    
    with open('/tmp/adf-fanspeed', 'w') as f:
        f.write(str(percentDiff) + '\n')
        f.close();
    
    pwmDuty = int(percentDiff * RPM_MAX / 100.0)
    
    print(myPID.out)
    wiringpi.pwmWrite(PWM_PIN, pwmDuty)
    return


def setup():
    wiringpi.wiringPiSetupGpio()
    wiringpi.pinMode(PWM_PIN, wiringpi.PWM_OUTPUT)
    
    wiringpi.pwmSetClock(768)  # Set PWM divider of base clock 19.2Mhz to 25Khz (Intel's recommendation for PWM FANs)
    wiringpi.pwmSetRange(RPM_MAX)  # Range setted
    
    wiringpi.pwmWrite(PWM_PIN, RPM_MAX)  # Setting to the max PWM
    return


def main():
    print("PWM FAN control starting")
    setup()
    setupTacho()
    
    while True:
        try:
            updateFanSpeed()
            readRPM()
            sleep(WAIT)
        except KeyboardInterrupt:
            fanOn()
            break
        except e:
            print("Something went wrong")
            print(e)
            fanOn()


if __name__ == "__main__":
    main()
