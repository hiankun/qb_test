"""
Mike Kroutikov, (c) 2014

Test QuickBot IR sensors.

Run this code on the BeagleBone side.

It will display for about 30 seconds the current values of all five sensors.

While this program runs, try putting an obstacle before each sensor. Unobstructed sensor should read
low value (e.g. 0). Placing palm of your hand at about 2 inches should cause large readings (300-700).

---------------------------------
Modified by hiankun.

The display time had been enlongate to about 300 seconds for longer testing period.

Also, the output had been changed from raw readings to centimeters.

Note that the unit conversion to centimeter was not adquite,
and further tunning is necessary.

"""
import Adafruit_BBIO.ADC as ADC
import time

if __name__ == '__main__':
    import config

    print "Testing IR sensors. Try putting hand in front of a sensor to see its value change"

    ADC.setup()

    for _ in range(600):
        values = tuple(ADC.read_raw(pin) for pin in config.IR_PINS)
        time.sleep(0.5)
	distances = map(lambda x: 2076.0/(x-11.0) if (x != 11.0) else nan, values)
	print '{:10.2f}{:10.2f}{:10.2f}{:10.2f}{:10.2f}'.format(*distances)

    print "Done"
