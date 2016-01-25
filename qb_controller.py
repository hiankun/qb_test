import contextlib
import time

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
    if left_speed > 100:
        left_speed = 100
    elif left_speed < -100:
        left_speed = -100

    if right_speed > 100:
        right_speed = 100
    elif right_speed < -100:
        right_speed = -100

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
    # if the value is not available, assign -999
    distances = map(lambda x: 2076.0/(x-11.0) if (x != 11.0) else -999., values)
    #print '{:10.2f}{:10.2f}{:10.2f}{:10.2f}{:10.2f}'.format(*distances)
    return distances

def do_control(obstacles):
    if (obstacles[1] < 5.0):
        run_motor(50,-50)
    elif (obstacles[3] < 5.0):
        run_motor(-50,50)
    else:
        run_motor(50,50)

if __name__ == '__main__':
    import config

    left_motor1, left_motor2, left_pwm = config.LEFT_MOTOR_PINS
    right_motor1, right_motor2, right_pwm = config.RIGHT_MOTOR_PINS

    setup_motor()

    ADC.setup()
    for _ in range(100):
        distances = get_ir_distance(config.IR_PINS)
        time.sleep(0.1)
        do_control(distances)

    run_motor(0,0)
    time.sleep(1)

    shutdown_motor()
