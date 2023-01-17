import robomaster
from robomaster import robot
from robomaster import camera
from multi_robomaster import multi_robot
import time
from math import sqrt

def cb(tup,*args , **kwargs):

    print(tup)
    return

class PID:
    def __init__(self, KP, KI, KD, target, max_v):
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
        self.last_time = time.time()
        self.max_v = max_v
        self.D = 0
    def calc(self, pos, target=None):
        """
        Berechnet den Output "Regler" des PID-Reglers und gibt diese zurück
        :param pos: Aktuelle Position als float
        :param timestep: Zeitdifferenz zwischen letztem und aktuellem Schritt als float
        :param target: Zielposition als float
        :return e: Reglerabweichung als float
        """
        # Fehlerberechnung
        #ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        t = time.time()
        if target != None:
            self.target = target
        timestep = t-self.last_time
        self.error = pos[0] - self.target
        if timestep != 0:
            self.integral_error = self.integral_error + self.error * timestep
            self.derivative_error = (self.error - self.lasterror) / timestep
        self.last_time = t
        # Berechnung der Reglerabweichung
        Regler = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error

        # Vorbereitung des nächten Reglerschrittes
        self.lasterror = self.error

        #Robotergeschwindigkeit setzen
        if Regler > self.max_v:
            Regler = self.max_v
        if Regler < -self.max_v:
            Regler = -self.max_v

        print(Regler)
        print(pos)
        if abs(self.error) > 2:
            ep_chassis.drive_speed(x=Regler, y=0, z=0, timeout=5)
        else:
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
if __name__ == '__main__':
    global pos
    pos = False
    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type='sta')
    except:
        exit()

    Regler = PID(0.001, 0.0001, 0.0005, 200, 0.2)
    cb_Distance = Regler.calc
    ep_chassis = ep_robot.chassis
    arm = ep_robot.robotic_arm
    ep_robot.sensor.sub_distance(callback=cb_Distance)
    #ep_chassis.drive_speed(x=-0.12, y=0, z=0, timeout=5)
    #a = ep_chassis.sub_velocity(freq=5, callback=cb)
    """
    time.sleep(3)
    time.sleep(3)

    
    time.sleep(3)
    ep_chassis.drive_speed(x=0.1, y=-0, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    time.sleep(3)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    ep_robot.close()
    """
    exit()