from controller import Robot, Motor, Device
import math
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
max_speed = 6.28
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
prox_sensors = []
for ind in range(8):
	sensor_name = 'ps'+str(ind)
	prox_sensors.append(robot.getDevice(sensor_name))
	prox_sensors[ind].enable(timestep)
	print("ind:{} , val: {}".format(ind,prox_sensors[ind].getValue()))
while (robot.step(timestep)!=-1):
	for ind in range(8):
		print("ind:{} , val: {}".format(ind,prox_sensors[ind].getValue()))
	right_wall = (math.floor(prox_sensors[2].getValue()) > 80)
	right_corner = (math.floor(prox_sensors[1].getValue()) > 80)
	front_wall = (math.floor(prox_sensors[0].getValue()) > 80)
	extra = (math.floor(prox_sensors[7].getValue()) > 80)
	if front_wall and extra and right_corner:
		left_speed = 0
		right_speed = 0
	elif right_wall:
		left_speed = max_speed
		right_speed = max_speed
	else:
		left_speed=max_speed
        right_speed=max_speed/10
	if(right_corner):
		left_speed=max_speed/8
        right_speed=max_speed
	left_motor.setVelocity(left_speed)
	right_motor.setVelocity(right_speed)

