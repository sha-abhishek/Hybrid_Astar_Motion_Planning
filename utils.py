# utils.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
import math
import numpy as np

def show(p):
    for i in p:
        print(i)   

class Value():
    """A helper class for adding f & g values to your PriorityQueue """

    def __init__(self, f, g):
        self.g = g
        self.f = f

class OrderedSet:
    """ An ordered list of elements """
    
    def __init__(self):
        self._container = []
    
    def add(self, item):
        if item in self._container:
            self._container.append(item)
        else:
            self._container.append(item)

    def has(self, item):
        return self._container.__contains__

    def remove(self, item):
        if item in self._container:
            self._container.remove(item)
    
    def clear(self):
        self._container.clear()
    
    def __contains__(self, item):
        return self._container.__contains__(item)

    def __len__(self):
        return self._container.__len__()
    
    def __iter__(self):
        return self._container.__iter__()
    
    def pop(self, last=True):
        if last:
            e = self._container.pop()
        else:
            e = self._container.pop(0)
        return e

class PriorityQueue:
    """
        A Queue in which the minimum (or maximum) element (as determined by f and
        order) is returned first.
    """
    def __init__(self, order=min, f=lambda v:v):
        if order == min or order == "min":
            self.order = min
        elif order == max or order == "max":
            self.order = max
        else:
            raise KeyError("order must be min or max")
        self.f = f

        self._dict = {}
  
    def get(self, item):
        return self._dict.__getitem__(item)

    def put(self, item, value):
        if item not in self._dict:
            self._dict[item] = value
        else:
            self._dict[item] = value

    def has(self, item):
        return self._dict.__contains__(item)

    def remove(self, item):
        if item in self._dict:
            del self._dict[item]

    def pop(self):
        if len(self._dict) > 0:
            tar = self.order(self._dict, key=lambda k: self.f(self._dict.get(k)))
            val = self._dict[tar]
            del self._dict[tar]
            return tar, val
        raise IndexError("pop from empty priority queue")

    def __iter__(self):
        return self._dict.__iter__()
    
    def __contains__(self, item):
        return self._dict.__contains__(item)

    def __len__(self):
        return self._dict.__len__()

    def __getitem__(self, key):
        return self._dict.__getitem__(key)
    
    def __setitem__(self, key, value):
        return self._dict.__setitem__(key, value)
    
    def __delitem__(self, key):
        return self._dict.__delitem__(key)
    

def discr_cor(safe_confs, cell_size=0.5):
    """
    Caclulates discretized coordinates (x,y) based on the 
    cell size of the grid for a given configuration
    """
    sf_x = safe_confs[0]
    sf_y = safe_confs[1]

    ds_x = math.ceil(sf_x / cell_size) - 1
    ds_y = math.ceil(sf_y / cell_size) - 1
    
    if sf_x % cell_size == 0:
        ds_x += 1
    if sf_y % cell_size == 0:
        ds_y += 1
        
    return (ds_x, ds_y)


def valid_config(loc, grid_dim): 
    """
    checks if a configuration lies outside the grid
    """
    conf = []
    x_min = grid_dim[0]
    y_min = grid_dim[1]
    x_max = grid_dim[2]
    y_max = grid_dim[3]
    for pt in loc:
        if pt[0] >= x_min and pt[0] <= x_max and pt[1] >= y_min and pt[1] <= y_max:
            conf.append(pt)            
    return conf


def aabb(conf,l=5,w=2):
    """
    Calculates AABB for a given configuration of robot
    """
    x = conf[0]
    y = conf[1]
    th = conf[2]
    rlx = x - (w/2)*math.sin(th)
    rly = y + (w/2)*math.cos(th)
    rrx = x + (w/2)*math.sin(th)
    rry = y - (w/2)*math.cos(th)
    frx = x + l*math.cos(th) + (w/2)*math.sin(th)
    fry = y + l*math.sin(th) - (w/2)*math.cos(th)
    flx = x + l*math.cos(th) - (w/2)*math.sin(th)
    fly = y + l*math.sin(th) + (w/2)*math.cos(th)

    A = [rlx,rly]
    B = [rrx,rry]
    C = [frx,fry]
    D = [flx,fly]

    xmin = min(rlx,rrx,frx,flx)
    ymin = min(rly,rry,fry,fly)
    xmax = max(rlx,rrx,frx,flx)
    ymax = max(rly,rry,fry,fly)

    return xmin,ymin,xmax,ymax


def aabb_col(conf,obs):     # obs = [[xmin,ymin,xmax,ymax],...]
    """
    checks AABB collision with obstacles for a given 
    robot configuration
    """
    rob_xmin,rob_ymin,rob_xmax,rob_ymax = aabb(conf)
    for j in range(len(obs)):
        o_xmin = obs[j][0]
        o_ymin = obs[j][1]
        o_xmax = obs[j][2]
        o_ymax = obs[j][3]

        if rob_xmin <= o_xmax and rob_xmax>= o_xmin:
            if rob_ymin <= o_ymax and rob_ymax >= o_ymin:
             return True
            
    return False

def plot_car(x, y, theta, length=5, width=2):
    """
    Plots car as a box given the configuration
    """
    # Define the four corners of the car with respect to the rear axle center
    x_corners = [0, length, length, 0]
    y_corners = [-width/2, -width/2, width/2, width/2]
    
    # Rotate the car by theta radians
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x_corners_rot = [x_corners[i]*cos_theta - y_corners[i]*sin_theta for i in range(4)]
    y_corners_rot = [x_corners[i]*sin_theta + y_corners[i]*cos_theta for i in range(4)]
    
    # Translate the car to the desired location
    x_corners_trans = [x + x_corners_rot[i] for i in range(4)]
    y_corners_trans = [y + y_corners_rot[i] for i in range(4)]
    
    return x_corners_trans, y_corners_trans