"""
IMPORTS
"""
import matplotlib.pyplot as plt
import numpy as np
from math import *
import random

def env_setup():
    plt.xlim(0,10)
    plt.ylim(0,10)
    obs1 = [8,8,1]
    obs2 = [3,5,0.75]
    obs3 = [6,1,0.5]
    c1 = plt.Circle((obs1[0],obs1[1]),radius = obs1[2],fc='k')
    c2 = plt.Circle((obs2[0],obs2[1]),radius = obs2[2],fc='k')
    c3 = plt.Circle((obs3[0],obs3[1]),radius = obs3[2],fc='k')
    plt.gca().add_patch(c1)
    plt.gca().add_patch(c2)
    plt.gca().add_patch(c3)
    root = [9,1,0]
    goal = [1,2,pi/2]
    plt.plot(root[0],root[1],'^g')
    plt.plot(goal[0],goal[1],"^k")


def plot_path(path_pts):
    env_setup()
    transposed_list = np.array(path_pts).T.tolist()
    x = transposed_list[0]
    y = transposed_list[1]

    plt.plot(x,y,'r')

def cal_dist(X_rnd,X_comp):
    x1 =  X_rnd[0]
    y1 =  X_rnd[1]
    x2 =  X_comp[0]
    y2 =  X_comp[1]
    d = sqrt((x1-x2)**2 + (y1-y2)**2)
    return d


def update(state,vel,st_ang):
    """
    dt is the update interval
    """
    dt = 0.1
    L = 1
    x = state[0]
    y = state[1]
    theta = state[2]

    x_new = x + vel*cos(theta)*dt
    y_new = y + vel*sin(theta)*dt
    theta_new = theta + ((vel*tan(st_ang))/L)*dt

    new_state = [x_new,y_new,theta_new]
    return new_state

def get_nearest(X_rnd,V):
    dlist = []
    for vertex in V:
        d = sqrt((X_rnd[0]-vertex[0])**2 + (X_rnd[1]-vertex[1])**2)
        dlist.append(d)

    minind = dlist.index(min(dlist))
    return minind

def local_planner(X_rnd,X_near,st_ang,vel,int_t):
    #extraction
    x_near = X_near[0]; y_near = X_near[1]; theta_near = X_near[2]
    x_rnd = X_rnd[0]; y_rnd = X_rnd[1]
    #formulate the six possible points
    dt = 0.1
    iter = int(int_t/dt)
    X_f = [X_near]
    X_b = [X_near]
    X_fr = [X_near]
    X_fl = [X_near]
    X_br = [X_near]
    X_bl = [X_near]
    for i in range(iter):
        X_f.append(update(X_f[-1],vel,0))
        X_b.append(update(X_b[-1],-vel,0))
        X_fr.append(update(X_fr[-1],vel,-st_ang))
        X_fl.append(update(X_fl[-1],vel,st_ang))
        X_br.append(update(X_br[-1],-vel,st_ang))
        X_bl.append(update(X_bl[-1],-vel,-st_ang))
    d_f = cal_dist(X_rnd,X_f[-1])
    d_b = cal_dist(X_rnd,X_b[-1])
    d_fr = cal_dist(X_rnd,X_fr[-1])
    d_fl = cal_dist(X_rnd,X_fl[-1])
    d_br = cal_dist(X_rnd,X_br[-1])
    d_bl = cal_dist(X_rnd,X_bl[-1])

    distances = [d_f,d_b,d_fr,d_fl,d_br,d_bl]
    ind = distances.index(min(distances))
    X_all = [X_f,X_b,X_fr,X_fl,X_br,X_bl]
    return X_all[ind]

def obstacle_check(path_pts):
    """
    a function which checks the path_pts obtained
    with obstacles and boundaries
    """
    obs1 = [8,8,1]
    obs2 = [3,5,0.75]
    obs3 = [6,1,0.5]
    flag = 0
    for pts in path_pts:
        d1 = sqrt((obs1[0]-pts[0])**2+(obs1[1]-pts[1])**2)
        d2 = sqrt((obs2[0]-pts[0])**2+(obs2[1]-pts[1])**2)
        d3 = sqrt((obs3[0]-pts[0])**2+(obs3[1]-pts[1])**2)

        if d1 <= obs1[2] or d2 <= obs2[2] or d3 <= obs3[2]:
            flag = 1
            break
        if pts[0]<=0 or pts[0]>=10 or pts[1]<=0 or pts[1]>=10:
            flag = 1
            break
    return flag

def goal_check(X_new,goal):
    x_new = X_new[0]
    y_new = X_new[1]
    theta_new =  X_new[2]

    x_g = goal[0]
    y_g = goal[1]
    theta_g = goal[2]
    flag = 0
    d = sqrt((x_new-x_g)**2 + (y_new-y_g)**2)
    # t_d = abs(theta_g - theta_new)
    if d<=0.1:# and# t_d <=0.1:
        print("Goal Reached")
        flag = 1

    return flag


def rrt_with_constraints():
    root = [9,1,0] #x,y,theta
    goal = [1,2,pi/2]
    V = []
    E = []
    paths = []
    iter = 1000
    st_ang = pi/7 #steering angle for minimum turning radius pi/7 and 0.1 hsa been working best
    int_t = 1 #integration time
    vel = 0.1
    k = 0
    while True:
        if k==0:
            V.append(root)
            k+=1
            continue
        if k==1:
            x_rnd = random.uniform(0,10)
            y_rnd = random.uniform(0,10)
            x_prev = root[0]
            y_prev = root[1]
            theta_prev = root[2]
            path_pts = local_planner([x_rnd,y_rnd],[x_prev,y_prev,theta_prev],st_ang,vel,int_t)
            #obstacle check
            if obstacle_check(path_pts) == 1:
                continue
            X_new = path_pts[-1]
            if goal_check(X_new,goal) == 1:
                break
            V.append(X_new)
            E.append([root,X_new])
            paths.append(path_pts)
            plot_path(path_pts)
            k+=1
            continue

        if random.uniform(0,100) < 50:
            x_rnd = random.uniform(0,10)
            y_rnd = random.uniform(0,10)
        else:
            x_rnd = goal[0]
            y_rnd = goal[1]

        nind = get_nearest([x_rnd,y_rnd],V)
        X_near = V[nind]
        X_rnd = [x_rnd,y_rnd]
        path_pts = local_planner(X_rnd,X_near,st_ang,vel,int_t)
        if obstacle_check(path_pts) == 1:
            continue
        X_new = path_pts[-1]
        if goal_check(X_new,goal) == 1:
            break
        V.append(X_new)
        E.append([V[-2],V[-1]])
        paths.append(path_pts)
        plot_path(path_pts)
        plt.pause(0.0000000000001)
    plt.show()
    print(E)





rrt_with_constraints()
