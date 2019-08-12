"""
IMPORTS
"""
import matplotlib.pyplot as plt
import numpy as np
import random
from math import *
import time

def env_setup():
    plt.xlim(0,10)
    plt.ylim(0,10)

def nearestneighbour(new_V,V):
    dlist = []
    x_new = new_V[0]
    y_new = new_V[1]
    for vertex in V:
        x =  vertex[0]
        y =  vertex[1]
        dlist.append(sqrt((x-x_new)**2 + (y-y_new)**2))

    minind = dlist.index(min(dlist))     #returns the minimum dist value index in V
    return minind

def plotline(E,i):
    env_setup()
    pt1 = E[i][0]
    pt2 = E[i][1]
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    plt.plot([x1,x2],[y1,y2],"g")

def get_new_node(X_rnd,X_near):
    x_rnd = X_rnd[0]
    y_rnd = X_rnd[1]
    x_near = X_near[0]
    y_near = X_near[1]
    trav_dist = 0.2
    theta = atan2(y_rnd - y_near, x_rnd - x_near)
    x_new = x_near + trav_dist*cos(theta)
    y_new = y_near + trav_dist*sin(theta)
    return ([x_new,y_new])


def rrt():
    tic =  time.time()
    print("time taken")

    iter = 300
    root = [5,5]
    V = [] #vertex list
    E = [] #edge list
    env_setup()
    for i in range(iter):
        if i == 0:
            V.append(root)
            continue
        if i == 1:
            x =  random.uniform(0,10)
            y =  random.uniform(0,10)
            nind = nearestneighbour([x,y],V)
            X_new = get_new_node([x,y],V[nind])
            E.append([V[i-1],X_new])
            V.append(X_new)

            plotline(E,i-1)
            continue
        x =  random.uniform(0,10)
        y =  random.uniform(0,10)
        nind = nearestneighbour([x,y],V)
        #get new node
        X_new = get_new_node([x,y],V[nind])
        E.append([V[nind],X_new])
        V.append(X_new)
        plotline(E,i-1)
        plt.pause(0.00000000000000000000001)

    toc = time.time()
    print(toc-tic)
    plt.show()

def plotting_together(E):
    for pts in E:
        pt1 =  pts[0]
        pt2 =  pts[1]
        x1 = pt1[0]
        x2 = pt2[0]
        y1 = pt1[1]
        y2 = pt2[1]
        print(pts)

        env_setup()
        plt.plot([x1,x2],[y1,y2])

def random_walk():
    root = [5,5]
    iter  = 300
    V = []
    E = []
    for i in range(iter):
        if i==0:
            V.append(root)
            continue
        if i==1:
            x =  random.uniform(0,10)
            y =  random.uniform(0,10)
            E.append([root,[x,y]])
            V.append([x,y])
            continue
        x =  random.uniform(0,10)
        y =  random.uniform(0,10)
        V.append([x,y])
        E.append([V[i-1],V[i]])
        plotline(E,i-1)
        plt.pause(0.000000001)



rrt()
