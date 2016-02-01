import contextlib
import time
import numpy as np

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC


def setup_motor():
    # Initialize GPIO pins
    GPIO.setup(left_motor1, GPIO.OUT)
    GPIO.setup(left_motor2, GPIO.OUT)
    GPIO.setup(right_motor1, GPIO.OUT)
    GPIO.setup(right_motor2, GPIO.OUT)

    # Initialize PWM pins: PWM.start(channel, duty, freq=2000, polarity=0)
    PWM.start(left_pwm, 0)
    PWM.start(right_pwm, 0)

def run_motor(left_speed, right_speed):
    limit = 80
    if left_speed > limit:
        left_speed = limit
    elif left_speed < -limit:
        left_speed = -limit

    if right_speed > limit:
        right_speed = limit
    elif right_speed < -limit:
        right_speed = -limit

    if left_speed > 0:
        GPIO.output(left_motor1, GPIO.LOW)
        GPIO.output(left_motor2, GPIO.HIGH)
        PWM.set_duty_cycle(left_pwm, abs(left_speed))
    elif left_speed < 0:
        GPIO.output(left_motor1, GPIO.HIGH)
        GPIO.output(left_motor2, GPIO.LOW)
        PWM.set_duty_cycle(left_pwm, abs(left_speed))
    else:
        GPIO.output(left_motor1, GPIO.LOW)
        GPIO.output(left_motor2, GPIO.LOW)
        PWM.set_duty_cycle(left_pwm, 0)

    if right_speed > 0:
        GPIO.output(right_motor1, GPIO.LOW)
        GPIO.output(right_motor2, GPIO.HIGH)
        PWM.set_duty_cycle(right_pwm, abs(right_speed))
    elif right_speed < 0:
        GPIO.output(right_motor1, GPIO.HIGH)
        GPIO.output(right_motor2, GPIO.LOW)
        PWM.set_duty_cycle(right_pwm, abs(right_speed))
    else:
        GPIO.output(right_motor1, GPIO.LOW)
        GPIO.output(right_motor2, GPIO.LOW)
        PWM.set_duty_cycle(right_pwm, 0)
    
def shutdown_motor():
    GPIO.cleanup()
    PWM.cleanup()

def simple_motor_test():
    print '====== Testing Quick Bot ======='
    speed = 50 # if the speed is too low, the motor might be motionless
    print 'Run forward'
    run_motor(speed, speed)
    time.sleep(2)
    print 'Run backward'
    run_motor(-speed, -speed)
    time.sleep(2)
    print 'Run right'
    run_motor(speed, -speed)
    time.sleep(2)
    print 'Run left'
    run_motor(-speed, speed)
    time.sleep(2)
    print 'Stop'
    run_motor(0, 0)
    time.sleep(2)

def get_ir_distance(ir_pins):
    values = tuple(ADC.read_raw(pin) for pin in ir_pins)

    """ NOTE: the distance should be calibrated,
        the following equations are just a workaround.
    """
    # if the value is not available, assign -999
    #distances = map(lambda x: 2076.0/(x-11.0) if (x != 11.0) else -999., values)
    distances = map(lambda x: 2076.0/(x) if (x != 0.0) else 999., values)
    #print '{:10.2f}{:10.2f}{:10.2f}{:10.2f}{:10.2f}'.format(*distances)
    return np.array(distances)

def get_direction():
    #w=np.array([0.0, 0.7 ,1.0 ,0.7, 0.0]) #weightings

    # IR sensors
    theta = np.array([90.0, 45.0, 0.0, -45.0, -90.0]) * np.pi/180.0 
    ir_sensor_vector = np.array([[np.sin(x), np.cos(x)] for x in theta]) 
    dist = get_ir_distance(config.IR_PINS)

    bearing = np.dot(dist, ir_sensor_vector)
    return bearing 


def do_control():
    """ NOTE: the controller is a simple one for test only.
        It should be replaced in the future.
    """

    bearing = get_direction()

    forward_speed = bearing[1] * 0.08
    lateral_speed = bearing[0] * 0.06

    left_wheel = forward_speed - lateral_speed
    right_wheel = forward_speed + lateral_speed

    print left_wheel, right_wheel
    run_motor(left_wheel, right_wheel)

    """
    if (obstacles[1] < 5.0):
        run_motor(50,-50)
    elif (obstacles[3] < 5.0):
        run_motor(-50,50)
    else:
        run_motor(50,50)
    """


if __name__ == '__main__':
    import config

    left_motor1, left_motor2, left_pwm = config.LEFT_MOTOR_PINS
    right_motor1, right_motor2, right_pwm = config.RIGHT_MOTOR_PINS

    setup_motor()

    ADC.setup()
    for _ in range(500):
        time.sleep(0.2)
        do_control()

    run_motor(0,0)
    time.sleep(1)

    shutdown_motor()
