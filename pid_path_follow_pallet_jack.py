from __future__ import division
import matplotlib.pyplot as plt
import math as m
import numpy as np
from draw_diff_drive import draw_robot
from drawing_pallet_jack import dpj
import copy

DRAW_PALLET = True
DRAW_DIFF = False

def update(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x = x + v*m.cos(theta+s)*dt
    y = y + v*m.sin(theta+s)*dt
    theta = wrapToPi(theta + (v/L)*m.sin(s)*dt)
    return(x,y,theta)

def wrapToPi(theta):
    return m.atan2(m.sin(theta),m.cos(theta))

def calc_perp(x,y,pt1,pt2):
    #this also needs to tell me if the xt,yt point has gone beyond pt2
    skip = False
    ld = 1
    #find the equation of the line
    x1,y1 = pt1
    x2,y2 = pt2
    try:
        phi = m.atan2((y2-y1),(x2-x1))
        slope = (y2-y1)/(x2-x1)
        intercept = y1 - slope*x1
        xp = (x + slope*y - slope*intercept)/(1+slope**2)
        yp = intercept + slope*xp
        xt = xp + m.cos(phi)
        yt = yp + m.sin(phi)
        #checking if it has crossed pt2
        check_angle = m.atan2((yt-y2),(xt-x2))
        if check_angle == phi:
            skip = True
        else:
            #everything is fine
            pass

    except ZeroDivisionError:
        xp = x1
        yp = y
        sign = (y2-y1)/abs(y2-y1)
        xt = xp
        yt = yp + sign*1
        #here also we need to check the crossing thresh
        if sign>0:
            if yt>y2:
                skip = True
        elif yt<y2:
            skip = True

    return(xp,yp,xt,yt,skip)

def calc_target(x,y,goal_points):
    if len(goal_points) >=3:
        goal_points = goal_points[0:3]
    else:
        pass
    best_dist = 999 #random
    best_pt = [x,y] #random
    best_target = [x,y] #random
    proximity = 999 #random
    for i in range(len(goal_points)-1):
        pt1 = goal_points[i]
        pt2 = goal_points[i+1]
        xp,yp,xt,yt,skip = calc_perp(x,y,pt1,pt2)
        perp_dist = m.sqrt((x-xp)**2 + (y-yp)**2)
        if perp_dist < best_dist:
            best_dist = perp_dist
            best_pt = goal_points[i]
            next_pt = goal_points[i+1]
            best_target[0] =xt
            best_target[1] =yt
            proximity = m.sqrt((next_pt[0]-xt)**2 + (next_pt[1]-yt)**2) #to pt2

    return (best_target,proximity,skip)

def seek_one(start,goal):
    x,y,theta = start
    Kp = 1
    Kpd = 0.1
    #thets calculations
    beta = m.atan2((goal[1]-y),(goal[0]-x))
    dist_error = m.sqrt((x-goal[0])**2 + (y-goal[1])**2)
    heading_error = wrapToPi(beta - theta)
    # print(m.degrees(heading_error),m.degrees(beta))
    s = Kp*heading_error
    if s > m.pi/2:
        s = m.pi/2
    if s<-m.pi/2:
        s = -m.pi/2
    # if abs(heading_error)>m.pi/2:
    #     s = -s
    #
    # v = Kpd*dist_error + 0.1
    x,y,theta = update(x,y,theta,v,s)
    return x,y,theta,s


# start = [2.5,10,0]
# goal_points = [[5,8],[10,5],[15,9],[20,1],[25,21],[1,20],[3,25]] #successfull path follow
# start = [0,0,0]
# goal_points = [[1,2],[2,4],[3,5],[4,7],[5,8]] # successfull path follow
start = [-2,-2,0]
goal_points = [[0,0],[1.98,2.375],[4.04,4.23],[6.28,5],[8.75,4.079],[10.51,2.45],[12,0],[14,-2]] #succesfull
# start = [0,0,0]
# goal_points = [[0,0],[2,2],[6,7],[9,10]] #succesfull
# start = [-2,-2,0]
# goal_points = [[0,0],[1,2],[1,-5],[5,6]]
dummy_gp = copy.deepcopy(goal_points)
goal = [15,9]
# goal_points = [[2,2]]
x,y,theta = start
v = 1
s = 0
gp_array = np.array(goal_points)
x_traj = []
y_traj = []

# plt.plot([start[0],gp_array[0,0]],[start[1],gp_array[0,1]])


#bad way to do it
# for xg,yg in goal_points:
#     while m.sqrt((x-xg)**2 + (y-yg)**2)>0.1:
#         plt.cla()
#         plt.axis('scaled')
#         plt.xlim(0,20)
#         plt.ylim(0,20)
#         plt.plot(gp_array[:,0],gp_array[:,1])
#         if DRAW_DIFF:
#             draw_robot(x,y,theta)
#         if DRAW_PALLET:
#             dpj(x,y,theta,s)
#
#         x_traj.append(x)
#         y_traj.append(y)
#         plt.plot(x_traj,y_traj,'--')
#         x,y,theta,s = seek_one([x,y,theta],[xg,yg])
#         plt.pause(0.0001)
skip = False
while len(dummy_gp) >1:
    #first step would be to find the target point
    target,proximity,skip = calc_target(x,y,dummy_gp)
    xt,yt = target
    if proximity<0.1 or skip==True:
        dummy_gp.pop(0)
    if skip==True:
        print(skip)
    plt.cla()
    plt.axis('scaled')
    plt.xlim(-5,15)
    plt.ylim(-5,15)
    # plt.plot(gp_array[:,0],gp_array[:,1])
    plt.plot(start[0],start[1],'co')
    plt.plot(xt,yt,'ro')

    if DRAW_DIFF:
        draw_robot(x,y,theta)
    if DRAW_PALLET:
        dpj(x,y,theta,s)

    x_traj.append(x)
    y_traj.append(y)
    plt.plot(x_traj,y_traj,'--')
    x,y,theta,s = seek_one([x,y,theta],[xt,yt])
    plt.pause(0.0001)





#make the speed explicitly 0 after this

plt.show()
