import os
import time
import board
from adafruit_motorkit import MotorKit

WHEEL_RADIUS = 0.067/2

if __name__ == "__main__":

	seconds = 1 # s
	vel_lin = 0.40 # m/s
	vel_ang = 0 # rad/s

	revolutions = (seconds * vel_lin) / (2 * 3.1415 * WHEEL_RADIUS)

	if vel_lin != 0 and vel_ang == 0:
		print("doing wheel revs={:.2f} in seconds={}".format(revolutions, seconds))
	if vel_ang != 0:
		print("doing spins={:.2f} in seconds={}".format(vel_ang * seconds / (2*3.1415), seconds))


	os.system("python3 set_motors.py {} {}".format(vel_lin, vel_ang))
	time.sleep(seconds)
	os.system("python3 set_motors.py 0 0")


	"""
	kit = MotorKit(i2c=board.I2C())

	kit.motor4.throttle = vel_ang * VEL_ANG_MULTIPLIER
	time.sleep(seconds)
	kit.motor4.throttle = 0
	"""

