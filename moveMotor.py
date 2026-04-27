# Simple demo of using auxiliary PWM pins on Raspi MotorHat

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time

# Initialise the PCA9685 using the default address (0x60) or (0x6F).
mh = Adafruit_MotorHAT(addr=0x60, i2c_bus=1) 

# Helper function to make setting a servo pulse width simpler.
# Pulse is in microseconds
# Pulse is in microseconds
def set_servo_pulse(channel, pulse):
    pulse_length = (1000000. / 50. ) / 4096.    # 1,000,000 us per second
                                                # 50 Hz PWM for MG90 servo motors
                                                # 12 bits of resolution
    pulse_width = int(float(pulse) / pulse_length)
    mh._pwm.setPWM(channel, 0, pulse_width)     # do not change 0 here.
    print(pulse_width)
    
# Set frequency to 50hz, which is spec for MG90. 
# You can not use Servo Motor and Stepper Motor on the same hat simultaneously 
# as the stepper motor requires a different frequency.
mh._pwm.setPWMFreq(50)

# There are only 4 channels available: 0, 1, 14 and 15
# Do not use any other channels.
# Move servo on channel O between extrems.
# MG90S is 500 to 2000 microseconds

set_servo_pulse(1,2200)

# print('Moving servo on channel 0, press Ctrl-C to quit...')
# while True:
#     set_servo_pulse(0,2500)
#     set_servo_pulse(1,2000)
#     set_servo_pulse(14,2500)
#     set_servo_pulse(15,2500)
    
#     time.sleep(1)
#     set_servo_pulse(0,500)
#     set_servo_pulse(1,500)
#     set_servo_pulse(14,500)
#     set_servo_pulse(15,500)
#     time.sleep(1)

    
