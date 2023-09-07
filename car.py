import numpy as np
from math import *

class Car():
    def __init__(self,l=5,w=2,max_phi=pi/3,v=1,dt=1):
        self.l = l
        self.w = w
        self.v = v
        self.dt = dt
        self.max_phi = max_phi
        pass
    
    def astar_step(self,cur_conf):
        """
        Kinematic Bicycle Model

        Parameters:
        x: float
            x-coordinate of the vehicle
        y: float
            y-coordinate of the vehicle
        yaw: float
            yaw angle of the vehicle (radians)
        v: float
            velocity of the vehicle
        delta: float
            steering angle of the front wheels (radians)
        L: float
            wheelbase of the vehicle
        dt: float
            time step

        Returns:
        x: float
            new x-coordinate of the vehicle
        y: float
            new y-coordinate of the vehicle
        yaw: float
            new yaw angle of the vehicle (radians)
        """
        x = cur_conf[0]
        y = cur_conf[1]
        yaw = cur_conf[2]
        # Calculate the curvature of the vehicle's path

        deltas = [-self.max_phi,0,self.max_phi]
        next_confs = []
        for i in range(len(deltas)):
            delta = deltas[i]
            curvature = tan(delta) / self.l
            xn = x
            yn = y
            yaw_n = yaw
            # Update the position and yaw angle of the vehicle
            dt1 = 0.001
            n = int(self.dt/dt1)
            for j in range(n):
                xn = xn + self.v * cos(yaw_n)*dt1 
                yn = yn + self.v * sin(yaw_n)*dt1
                yaw_n = yaw_n + self.v * curvature * dt1

            # yaw_n = yaw + self.v * curvature * self.dt
            # xn = x + sin(yaw_n)/curvature
            # yn = y + cos(yaw_n)/curvature
            


            next_confs.append((xn,yn,yaw_n))

        return next_confs
