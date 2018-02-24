#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

GPIO.setmode(GPIO.BOARD)

GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

try:
    while True:
        sw1 = GPIO.input(16)
        sw2 = GPIO.input(18)

        if sw1 == 1:
            pwm.set_pwm(1, 0, 600)
        else:
            pwm.set_pwm(1, 0, 375)

        if sw2 == 1:
            pwm.set_pwm(2, 0, 600)
        else:
            pwm.set_pwm(2, 0, 375)


except KeyboardInterrupt:
    pass
