import sys
import board
from adafruit_motorkit import MotorKit


WHEEL_RADIUS = 0.067/2
ROBOT_RADIUS = 0.178/2

BATTERY_CHARGE = 5.38 / (1.5*6)

VEL_ANG_MULTIPLIER = 0.047 / BATTERY_CHARGE

MIN_ANG = 7 # rad/s  (at 5.3v)
MAX_ANG = 3

MIN_VEL = 0.2 # m/s  (at 5.3v)
MAX_VEL = 0.4


if __name__ == "__main__":
	kit = MotorKit(i2c=board.I2C())

	try:
	# if True:
		linear = float(sys.argv[1]) # x translation, m/s
		angular = float(sys.argv[2]) # z rotation, rad/s
		if angular != 0:
			# Prioritize turning over moving straight
			linear = 0

		vel_ang_linear = linear / WHEEL_RADIUS
		vel_ang_angular = angular * ROBOT_RADIUS / WHEEL_RADIUS
		# print("vel_ang_linear={:.2f}, vel_ang_angular={:.2f}".format(vel_ang_linear, vel_ang_angular))
		print("max speed={:.2f}".format((vel_ang_linear+vel_ang_angular)*VEL_ANG_MULTIPLIER))

		kit.motor1.throttle = (vel_ang_linear - vel_ang_angular) * VEL_ANG_MULTIPLIER
		kit.motor4.throttle = (vel_ang_linear + vel_ang_angular) * VEL_ANG_MULTIPLIER

	except:
	# else:
		kit.motor1.throttle = 0
		kit.motor4.throttle = 0

