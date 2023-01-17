from robomaster import robot
from robomaster import camera
from multi_robomaster import multi_robot
import time
import math
import numpy as np
import matplotlib.pyplot as plt


class PID:
    def __init__(self, KP, KI, KD, target):
        """
        Diese Klasse dient als eine einfache Implementierung des PID-Reglers
        :param KP: Koeffizent des P-Anteils als float
        :param KI: Koeffizient des I-Anteils als float
        :param KD: Koeffizent des D-Anteils als float
        :param target: Zielposition des Reglers als float
        """
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.error = 0
        self.lasterror = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.target = target

    def calc(self, pos, timestep, target=None):
        """
        Berechnet den Output "Regler" des PID-Reglers und gibt diese zurück
        :param pos: Aktuelle Position als float
        :param timestep: Zeitdifferenz zwischen letztem und aktuellem Schritt als float
        :param target: Zielposition als float
        :return e: Reglerabweichung als float
        """
        # Fehlerberechnung
        if target != None:
            self.target = target
        self.error = self.target - pos
        self.integral_error += self.error * timestep
        self.derivative_error = (self.error - self.lasterror) / timestep

        # Berechnung der Reglerabweichung
        Regler = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
        # Vorbereitung des nächten Reglerschrittes
        self.lasterror = self.error
        return Regler


class Testcase():
    def __init__(self):
        self.w = 7
        self.t = np.linspace(0, 10, 100)
        self.x_wo_controller = []
        self.x= []
        for i in range(0, 10):
            self.x_wo_controller.append(7)
        for i in range(10, 30):
            self.x_wo_controller.append(10)
        for i in range(30, 40):
            self.x_wo_controller.append(5)
        for i in range(40, 70):
            self.x_wo_controller.append(7)
        for i in range(70, 100):
            self.x_wo_controller.append(8)
        self.x_wo_controller = np.array(self.x_wo_controller)
    def controlled_system(self,x, Reglerwert, dt, z=0):
        x += Reglerwert * dt
        x = x + z
        return x

    def loop(self):
        controller = PID(1, 1, 1, 7)
        dt = self.t[1] - self.t[0]
        for t_step in self.t:
            # Berechnung der Regelgröße
            if t_step == 0:
                x = 7

            elif 1.02 > t_step > 1:
                x = 10
            elif 3.04 > t_step > 3:
                x = 5
            elif 4.05 > t_step > 4:
                x = 7
            elif 7.09 > t_step > 7:
                x = 8

            # Berechnung der Regelstrecke
            y = controller.calc(x, dt)
            x = self.controlled_system(x, y, dt)
            self.x.append(x)
        self.x = np.array(self.x)
    def display(self):

        plt.plot(self.t, self.x_wo_controller,color="red")
        plt.plot(self.t, self.x, color="green")
        plt.show()

class Robot_Sim():
    def __init__(self):
        self.w = 7
        self.t = np.linspace(0, 10, 100)
        self.x_wo_controller = []
        self.x = []
        for i in range(0, 10):
            self.x_wo_controller.append(7)
        for i in range(10, 30):
            self.x_wo_controller.append(10)
        for i in range(30, 40):
            self.x_wo_controller.append(5)
        for i in range(40, 70):
            self.x_wo_controller.append(7)
        for i in range(70, 100):
            self.x_wo_controller.append(8)
        self.x_wo_controller = np.array(self.x_wo_controller)

    def controlled_system(self,x, Reglerwert, dt, z=0):
        T = 0.256
        Tt = 0.28
        K = 1
        x_dot = Reglerwert*K*(1-math.exp(-1*(dt-Tt)/T))
        x += x_dot * dt
        x = x + z
        return x

    def loop(self):
        controller = PID(0.1, 0, 0, 7)
        dt = self.t[1] - self.t[0]
        for t_step in self.t:
            # Berechnung der Regelgröße
            if t_step == 0:
                x = 7

            elif 1.02 > t_step > 1:
                x = 10
            elif 3.04 > t_step > 3:
                x = 5
            elif 4.05 > t_step > 4:
                x = 7
            elif 7.09 > t_step > 7:
                x = 8

            # Berechnung der Regelstrecke
            y = controller.calc(x, dt)
            x = self.controlled_system(x, y, dt)
            self.x.append(x)
        self.x = np.array(self.x)
    def display(self):
        plt.plot(self.t, self.x_wo_controller, color="red")
        plt.plot(self.t, self.x, color="green")
        plt.show()

t = Robot_Sim()
t.loop()
t.display()
