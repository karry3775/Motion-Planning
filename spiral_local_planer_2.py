"""
IMPORTS
"""
import matplotlib.pyplot as plt
import math
from scipy import *
from sympy import Symbol as sym
from sympy import integrate, cos , diff , sin
from sympy import *
import time
"""
FUNCTIONS
"""
def env_setup():
    plt.xlim(-10,10)
    plt.ylim(-10,10)

def posture():
    """
    Posture is defined by [x,y,theta,k]
    x,y: cordinates
    theta: heading
    k : curvature
    """
    start = [1,2,2,2]
    goal = [8,7,pi/2,2]
    return(start,goal)


def get_coeff(p):
    p1 = p[0]; p2 = p[1]; p4 = p[2]
    start,goal = posture()
    p0 = start[-1]
    p3 = goal[-1]

    a0 = p0
    a1 = -(5.5*p0 - 9*p1 + 4.5*p2 - p3)/p4
    a2 = (9*p0 - 22.5*p1 + 18*p2 - 4.5*p3)/(p4**2)
    a3 = (-4.5*p0 - 13.5*p1 + 13.5*p2 - 4.5*p3)/(p4**2)
    return (a0,a1,a2,a3)


def bending_energy(p):
    a0,a1,a2,a3 = get_coeff(p)
    s = sym('s')
    result = integrate((a0 + a1*s + a2*(s**2) + a3*(s**3))**2,s)
    return result

def theta_penalty(p):
    """
    unweighted
    """
    a0,a1,a2,a3 = get_coeff(p)
    start,goal = posture()
    tht0 = start[2]
    sf = p[2]
    s = sym('s')
    thtsf = tht0 + (a3/4)*(s**4) + (a2/3)*(s**3) + (a1/2)*(s**2) + a0*s
    thtf = goal[2]
    tht_penalty = abs(thtsf.subs(s,sf) - thtf)
    return (thtsf,tht_penalty)

def x_penalty(p):
    a0,a1,a2,a3 = get_coeff(p)
    start,goal = posture()
    s = sym('s')
    #need to call theta_penalty
    thtsf,th_penalty = theta_penalty(p)
    sf = p[2]
    x0 =  start[0]
    xsf = x0 + (sf/24)*( \
    cos(thtsf.subs(s,0)) + 4*cos(thtsf.subs(s,sf/8))  + \
    2*cos(thtsf.subs(s,(2*sf)/8)) + 4*cos(thtsf.subs(s,(3*sf)/8)) + \
    2*cos(thtsf.subs(s,(4*sf)/8)) + 4*cos(thtsf.subs(s,(5*sf)/8)) + \
    2*cos(thtsf.subs(s,(6*sf)/8)) + 4*cos(thtsf.subs(s,(7*sf)/8)) + \
    cos(thtsf.subs(s,(8*sf)/8)))

    xf = goal[0]
    x_p = abs(xsf-xf)
    return x_p

def y_penalty(p):
    a0,a1,a2,a3 = get_coeff(p)
    start,goal = posture()
    s = sym('s')
    #need to call theta_penalty
    thtsf,th_penalty = theta_penalty(p)
    sf = p[2]
    y0 =  start[1]
    ysf = y0 + (sf/24)*( \
    sin(thtsf.subs(s,0)) + 4*sin(thtsf.subs(s,sf/8))  + \
    2*sin(thtsf.subs(s,(2*sf)/8)) + 4*sin(thtsf.subs(s,(3*sf)/8)) + \
    2*sin(thtsf.subs(s,(4*sf)/8)) + 4*sin(thtsf.subs(s,(5*sf)/8)) + \
    2*sin(thtsf.subs(s,(6*sf)/8)) + 4*sin(thtsf.subs(s,(7*sf)/8)) + \
    sin(thtsf.subs(s,(8*sf)/8)))

    yf = goal[1]
    y_p = abs(ysf-yf)
    return y_p


def objective_function(p):
    """
    BENDING ENERGY + PENALTIES
    """
    #bending energy
    p1 = p[0]; p2 = p[1]; p4 = p[2]
    start,goal = posture()
    p0 = start[-1]
    p3 = goal[-1]

    a0 = p0
    a1 = -(5.5*p0 - 9*p1 + 4.5*p2 - p3)/p4
    a2 = (9*p0 - 22.5*p1 + 18*p2 - 4.5*p3)/(p4**2)
    a3 = (-4.5*p0 - 13.5*p1 + 13.5*p2 - 4.5*p3)/(p4**2)

    s = sym('s')
    result = integrate((a0 + a1*s + a2*(s**2) + a3*(s**3))**2)
    bending_energy = result.subs(s,p4) - result.subs(s,0)

    #weights
    alpha = 0.1
    beta = 0.1
    gamma = 0.1

    #theta penalty
    tht0 = start[2]
    sf = p4
    thtsf = tht0 + (a3/4)*(s**4) + (a2/3)*(s**3) + (a1/2)*(s**2) + a0*s #obtained symbolic
    thtf = goal[2]
    theta_penalty = gamma*abs(thtsf.subs(s,sf) - thtf)

    #x_penalty
    x0 =  start[0]
    xsf = x0 + (sf/24)*( \
    cos(float(thtsf.subs(s,0))) + 4*cos(float(thtsf.subs(s,sf/8)))  + \
    2*cos(float(thtsf.subs(s,(2*sf)/8))) + 4*cos(float(thtsf.subs(s,(3*sf)/8))) + \
    2*cos(float(thtsf.subs(s,(4*sf)/8))) + 4*cos(float(thtsf.subs(s,(5*sf)/8))) + \
    2*cos(float(thtsf.subs(s,(6*sf)/8))) + 4*cos(float(thtsf.subs(s,(7*sf)/8))) + \
    cos(float(thtsf.subs(s,(8*sf)/8))))

    xf = goal[0]
    x_penalty = alpha*(abs(xsf-xf))

    #y_penalty
    y0 =  start[1]
    ysf = y0 + (sf/24)*( \
    sin(float(thtsf.subs(s,0))) + 4*sin(float(thtsf.subs(s,sf/8)))  + \
    2*sin(float(thtsf.subs(s,(2*sf)/8))) + 4*sin(float(thtsf.subs(s,(3*sf)/8))) + \
    2*sin(float(thtsf.subs(s,(4*sf)/8))) + 4*sin(float(thtsf.subs(s,(5*sf)/8))) + \
    2*sin(float(thtsf.subs(s,(6*sf)/8))) + 4*sin(float(thtsf.subs(s,(7*sf)/8))) + \
    sin(float(thtsf.subs(s,(8*sf)/8))))

    yf = goal[1]
    y_penalty = alpha*(abs(ysf-yf))

    objective_value = bending_energy + x_penalty + y_penalty + theta_penalty
    print("breakdown:")
    print("total objective_value",objective_value)
    print("bending energy",bending_energy)
    print("x_penalty",x_penalty)
    print("y_penalty",y_penalty)
    print("theta_penalty",theta_penalty)
    return objective_value

def jacobian(p):
    """
    Returns the jacobian
    PS: p will be symbolic here
    """
    be = bending_energy(p)
    _, tht_p = theta_penalty(p)
    x_p = x_penalty(p)
    y_p = y_penalty(p)

    obj = be + tht_p + x_p + y_p
    first = diff(obj,p1)
    second = diff(obj,p2)
    third = diff(obj,p4)
    return(first,second,third)


def spiral_planner():
    start, goal = posture()
    p = [1,1,3] #the p values will originally come from optimization
    # objective_function(p)
    be = bending_energy(p)
    _, tht_p = theta_penalty(p)
    x_p = x_penalty(p)
    y_p = y_penalty(p)

    objective_value = be + tht_p + x_p + y_p

    print(objective_value)
    print(be)


tic = time.time()
p1 = sym('p1');p2 = sym('p2') ; p4 = sym('p4')
p = [p1, p2 ,p4]
jacobian(p)
toc = time.time()
print(toc-tic)
