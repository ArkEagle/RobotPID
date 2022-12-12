import robomaster
from robomaster import robot
from robomaster import camera
from multi_robomaster import multi_robot
import time
from math import sqrt

def cb(tup,*args , **kwargs):
    print(tup)
    return

if __name__ == '__main__':
    global pos
    pos = False
    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type='sta')
    except:
        exit()
    ep_chassis = ep_robot.chassis
    arm = ep_robot.robotic_arm
    x_val = 0.1
    y_val = 0.3
    z_val = 30
    call_pos = cb
    call_arm = cb
    a = ep_chassis.sub_position(cs=0, freq=5, callback=call_pos)
    time.sleep(3)
    time.sleep(3)

    ep_chassis.drive_speed(x=-0.1, y=-0.1, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0.1, y=-0, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0, y=0.1, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    ep_robot.close()
    exit()