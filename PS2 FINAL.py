from controller import Robot, Motor, DistanceSensor, Camera
import numpy as np
import math
import cv2
robot = Robot()
camera = robot.getDevice('camera')
camera.enable(1)
timestep = int(robot.getBasicTimeStep())
t = 1
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
max_speed = 6.28
target = 1
ps = []
for i in range(8):
    name = 'ps' + str(i)
    ps.append(robot.getDevice(name))
    ps[i].enable(timestep)
while robot.step(timestep) != -1:
    camera.saveImage('rgb.jpg', 100)
    img = cv2.imread('rgb.jpg')
    cv2.imshow('original', img)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if(target == 1):
        lower_color = np.array([60, 40, 40])
        upper_color = np.array([100, 255, 255])
    if(target == 2):
        lower_color = np.array([115, 95, 95])
        upper_color = np.array([140, 255, 255])
    if(target == 3):
        lower_color = np.array([135, 55, 45])
        upper_color = np.array([162, 253, 255])
    if(target == 4):
        lower_color = np.array([164, 125, 0])
        upper_color = np.array([178, 253, 255])
    rgb = cv2.medianBlur(rgb, 3)
    mask_color = cv2.inRange(rgb, lower_color, upper_color)
    det_color = cv2.bitwise_and(img, img, mask=mask_color)
    contours, garbage = cv2.findContours(
        mask_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.imshow('segment', det_color)
    cv2.waitKey(1)
    if(len(contours) > 0):
        t=1
        contours = sorted(
            contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if(cv2.contourArea(contours[0]) > 500):
            L = cv2.moments(contours[0])
            if(L['m00'] != 0):
                cx = int(L['m10']/L['m00'])
                cy = int(L['m01']/L['m00'])
                print(cx, cv2.contourArea(contours[0]), len(contours))
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)
        print("target {}".format(target))
        print("Sensor 0 : {}".format(ps[0].getValue()))
        print("Sensor 7 : {}".format(ps[7].getValue()))
        print("Sensor 6 : {}".format(ps[6].getValue()))
        print("Sensor 1 : {}".format(ps[1].getValue()))
        s0 = (math.floor(ps[0].getValue()) > 80)
        s1 = (math.floor(ps[1].getValue()) > 80)
        s7 = (math.floor(ps[7].getValue()) > 80)
        s6 = (math.floor(ps[6].getValue()) > 80)
        if (s7 or s0 or s1 or s6):
            if target != 4:
                target = target+1
                for i in range(40):
                    left_motor.setVelocity(-6.28)
                    right_motor.setVelocity(-6.28)
                    robot.step(timestep)
                for i in range(30):
                    left_motor.setVelocity(max_speed/10)
                    right_motor.setVelocity(max_speed)
                robot.step(timestep)
            else:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                break
    else:
        ls = -max_speed
        rs = max_speed
        t += 1
        if(t > 300):
            ls = max_speed/10
            rs = max_speed
        left_motor.setVelocity(ls)
        right_motor.setVelocity(rs)

    pass
