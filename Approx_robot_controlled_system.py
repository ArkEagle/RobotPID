from robomaster import robot
from robomaster import camera
from multi_robomaster import multi_robot
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import random
global v_array
def Gauss_Newton_Fit_PT1_Totzeit(t_array,y_array,filename):
    K = 1
    T = 1
    Tt = 1

    a = np.array([[K], [T], [Tt]])
    daempfung = 0.5

    for n in range(1000):
        F = np.zeros((len(t_array), 1))
        for n in range(len(t_array)):
            F[n] = y_array[n]-a[0]*(1-math.exp(-(t_array[n]-a[2])/a[1]))

        Resi = np.sum(np.power(F, 2))
        if Resi < 0.001:
            break

        J = np.zeros((len(t_array), 3))
        for n in range(len(t_array)):
            J[n][0] = 1-math.exp(-(t_array[n]-a[2])/a[1])
            J[n][1] = -a[0]/a[1]**2*(t_array[n]-a[2])*math.exp(-(t_array[n]-a[2])/a[1])
            J[n][2] = -a[0]/a[1]*math.exp(-(t_array[n]-a[2])/a[1])

        JTJ = np.dot(np.transpose(J), J)
        JTF = np.dot(np.transpose(J), F)
        da = np.linalg.solve(JTJ, JTF)
        dae_da = np.dot(daempfung, da)
        a = np.add(a, dae_da)

    f = np.zeros((len(t_array), 1))
    for n in range(len(t_array)):
        f[n] = a[0]*(1-math.exp(-(t_array[n]-a[2])/a[1]))
    print(f"Residuum:{Resi:.5f}")
    plt.plot(t_array, y_array, color="black", marker="o", linestyle ="None")
    plt.plot(t_array, f, color="green")
    plt.show()
    #Schreiben vorbereiten
    if '.csv' not in filename:
        filename = filename+'.csv'
    #Schreiben
    with open(filename, 'w') as file:
        writer = csv.writer(file,delimiter=';')
        writer.writerow(["K","T","Tt"])
        writer.writerow([a[0][0],a[1][0],a[2][0]])

def Test():
    # Testfall
    t = np.linspace(0, 10, 100)
    y = np.zeros((len(t), 1))
    a = [0.5, 2, 0.1]
    for n in range(len(t)):
        y[n] = a[0]*(1-math.exp(-(t[n]-a[2])/a[1]))+random.randint(-10, 10)/1000

    Gauss_Newton_Fit_PT1_Totzeit(t,y,'Coeff')

def Testdrive_x(n,t,v_x,filename):
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    time.sleep(t)
    for i in range(n):
        ep_chassis.drive_speed(x=v_x, y=0, z=0, timeout=5)
        time.sleep(t)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(t)

    # Daten aufbereiten
    t_array = np.linspace(0, t * n * 3, len(v_array))
    vx = []
    for e in v_array:
        vx.append(e[3])
    upperlimit = np.where(t_array >= 2 * t)[0][0]
    lowerlimit = np.where(t_array >= t)[0][0]

    vx_cut = vx[lowerlimit - 1:upperlimit + 1]
    t_calc = np.linspace(0, t, len(vx_cut))
    Gauss_Newton_Fit_PT1_Totzeit(t_calc, vx_cut, filename)

def cb(tup,*args , **kwargs):
    global v_array
    v_array.append(tup)
    return


if __name__ == '__main__':
    global v_array
    v_array = []

    # Initialisierung der Parameter
    t = 2
    n = 1
    freq = 10
    v = 0.3


    #Mit Roboter verbinden
    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type='sta')
    except:
        exit()
    ep_chassis = ep_robot.chassis
    arm = ep_robot.robotic_arm

    #Datensammeln
    call_vel = cb
    ep_chassis.sub_velocity(freq=5, callback=call_vel)
    v = 3/10
    filename = 'Coeff_x_'+str(v)
    Testdrive_x(n, t, v, filename)

    ep_robot.close()
    exit()
