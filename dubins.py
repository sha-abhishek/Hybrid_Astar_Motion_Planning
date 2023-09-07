import math
import numpy as np
import matplotlib.pyplot as plt

def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def LSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb
    
    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    return t, p, q, mode


def RSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)
    return t, p, q, mode

def LSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)
    return t, p, q, mode


def RSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    return t, p, q, mode

def dubins_path_planning_from_origin(ex, ey, eyaw, c):
    dx = ex
    dy = ey
    D = math.sqrt(dx ** 2.0 + dy ** 2.0)
    d = D * c

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(- theta)
    beta = mod2pi(eyaw - theta)
    
    planners = [LSL, RSR, LSR, RSL]

    bcost = float("inf")
    bt, bp, bq, bmode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if bcost > cost:
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

    px, py, pyaw = generate_course([bt, bp, bq], bmode, c)
    return px, py, pyaw, bmode, bcost


def dubins_path_planning(start,end, c):
    """
    Dubins path plannner
    input: start point, end point and max curvature(based on steering angle)
    output:
        px,py,pyaw,mode
    """
    sx = float(start[0])
    sy = float(start[1])
    syaw = float(start[2])
    ex = float(end[0])
    ey = float(end[1])
    eyaw = float(end[2])
    c = float(c)
    ex = ex - sx
    ey = ey - sy

    lex = math.cos(syaw) * ex + math.sin(syaw) * ey
    ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
    leyaw = eyaw - syaw

    lpx, lpy, lpyaw, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c)

    px = [math.cos(-syaw) * x + math.sin(-syaw)
          * y + sx for x, y in zip(lpx, lpy)]
    py = [- math.sin(-syaw) * x + math.cos(-syaw)
          * y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]

    return px, py, pyaw, mode, clen

def generate_course(length, mode, c):

    px = [0.0]
    py = [0.0]
    pyaw = [0.0]

    for m, l in zip(mode, length):
        pd = 0.0
        if m == "S":
            d = (1.0 * c)
        else:  # resolution for points in curvature  
            d = np.deg2rad(6.0)

        while pd < abs(l - d):
            px.append(px[-1] + d / c * math.cos(pyaw[-1]))
            py.append(py[-1] + d / c * math.sin(pyaw[-1]))

            if m == "L":  # left
                pyaw.append(pyaw[-1] + d)
            elif m == "S":  # straight
                pyaw.append(pyaw[-1])
            elif m == "R":  # right
                pyaw.append(pyaw[-1] - d)
            pd += d

        d = l - pd
        px.append(px[-1] + d / c * math.cos(pyaw[-1]))
        py.append(py[-1] + d / c * math.sin(pyaw[-1]))

        if m == "L":  # left
            pyaw.append(pyaw[-1] + d)
        elif m == "S":  # straight
            pyaw.append(pyaw[-1])
        elif m == "R":  # right 
            pyaw.append(pyaw[-1] - d)
        pd += d
    return px, py, pyaw

def dubin_path(start, end):  ### Add the angle either in np or theta 
    curvature = math.tan(np.deg2rad(30))/2
    px, py, pyaw, mode, clen = dubins_path_planning(start, end, curvature)
    
    # pyaw_2pi = []
    # for i in range(len(pyaw)):
    #     if pyaw[i] < 0:
    #         pyaw[i] += 2*math.pi
    #     pyaw_2pi.append(pyaw[i])
                    
    path = list(zip(px, py, pyaw))
    return path, len(path)

# start = [0,0,0]
# end = [5,9,3*(np.pi/2)]
# curvature = math.tan(np.deg2rad(30))/2

# path, num = dubin_path(start, end, curvature)
# print(num)