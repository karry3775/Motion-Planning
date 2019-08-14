########################
# PURE PURSUIT BASED MOVE TO POSE
# AUTHOR: KARTIK PRAKASH
# DATE: Jul/17/2019
########################
from __future__ import division
import math
import matplotlib.pyplot as plt
import copy
import random
import numpy as np
import sys, os
import time

from drawing_pallet_jack import dpj

ROTATE_TO_ANGLE = True
ALIGN_TO_LINE = True
TRANSLATE_TO_GOAL = False
MOVE_TO_GOAL = True

PAUSE = 0.001
LD_OFFSET = 0.5
DISABLE_SIGN = False

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')
# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

def one_time_password(x,y,theta,goal):
    goal_array = np.array([goal[0],goal[1]]).T
    prev_state = np.array([x,y,1]).T

    #find the Transformation matrix T
    tg = goal[2]
    goal_for_T = -np.matmul(([[math.cos(tg),math.sin(tg)],[-math.sin(tg),math.cos(tg)]]),goal_array)
    px,py = goal_for_T

    T = np.array([[math.cos(tg),math.sin(tg),px],
                  [-math.sin(tg),math.cos(tg),py],
                  [0,0,1]])

    transformed_state = np.matmul(T,prev_state)
    x_new = transformed_state[0]
    y_new = transformed_state[1]
    theta_new = wrapToPi(theta-goal[2])

    enablePrint()
    print("[INFO] x_new: {}, y_new: {}, theta_new: {}".format(x_new,y_new,theta_new))
    blockPrint()

    if theta_new == 0:
        dec = False
    elif y_new>0:
        if theta_new>0:
            dec = True
        else:
            dec = False
    else:
        if theta_new>0:
            dec = False
        else:
            dec = True
    return dec

def plot_setup(x,y):
    # plt.xlim(0,20)
    # plt.ylim(0,20)
    plt.xlim(x-10,x+10)
    plt.ylim(y-10,y+10)

    plt.axes().set_aspect('equal','datalim')
    # plt.axes().set_facecolor('xkcd:salmon')
    plt.axes().set_facecolor('tab:olive')

def update(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x+= v*math.cos(theta+s)*dt
    y+= v*math.sin(theta+s)*dt
    theta+= (v/L)*math.sin(s)*dt

    return (x,y,theta)

def wrapToPi(theta):
    return math.atan2(math.sin(theta),math.cos(theta))

def closest_pt(pt1,pt2,x,y,ld,dec): #pt1 = [A,B], pt2 = [P,Q], (x,y) are the current state
    if dec==False:
        A = pt1[0]
        B = pt1[1]
        P = pt2[0]
        Q = pt2[1]
    else:
        A = pt2[0]
        B = pt2[1]
        P = pt1[0]
        Q = pt1[1]

    try:
        m = (Q-B)/(P-A)
        c = B - m*A
        xp = (x + m*(y-c))/(1 + m**2)
        yp = m*xp + c
        phi = math.atan2(Q-B,P-A)
        xg,yg,ld = calc_target(xp,yp,x,y,phi,ld)
    except ZeroDivisionError:
        print("Inside ZDE")
        xp = A
        yp = y
        pd = math.sqrt((xp-x)**2 + (yp-y)**2)
        ld = pd + LD_OFFSET #"""changing ld manually """
        dely = math.sqrt(abs(ld**2 - pd**2))
        #calculate sign for forward movement in path
        if Q-B>0:
            sign = 1
        else:
            sign = -1
        xg = xp
        yg = yp + sign*dely

    # plt.plot([pt1[0],pt2[0]],[pt1[1],pt2[1]],'b',label='path')
    return(xp,yp,xg,yg,ld)

def plot_theta(start,goal):
    plt.arrow(start[0],start[1],math.cos(start[2]),math.sin(start[2]),width=0.1,color='r')
    plt.arrow(goal[0],goal[1],math.cos(goal[2]),math.sin(goal[2]),width=0.1,color='g')

def calc_target(xp,yp,x,y,phi,ld):
    #calculate pd which is the perpendicular distance
    pd = math.sqrt((xp-x)**2 + (yp-y)**2)
    ld = pd + LD_OFFSET#"""changing ld manually """
    #calcuulate offset
    off = math.sqrt(abs(ld**2 - pd**2))
    #calculate the del values for x and y
    delx = off*math.cos(phi)
    dely = off*math.sin(phi)
    #calculate xg and yg
    xg = xp + delx
    yg = yp + dely
    return(xg,yg,ld)

def draw_vehicle(x,y,theta):
    #base drawing
    off = 0# -0.25
    a = np.array([1,0,1]).T
    b = np.array([0,-1,1]).T
    c = np.array([0,1,1]).T
    d = np.array([-0.1,1,1]).T
    e = np.array([-0.1,0.5,1]).T
    f = np.array([-0.1,-0.5,1]).T
    g = np.array([-0.1,-1,1]).T
    i = np.array([-2,1,1]).T
    j = np.array([-2,0.5,1]).T
    k = np.array([-2,-0.5,1]).T
    l = np.array([-2,-1,1]).T



    T = transform_mat(x,y,theta)
    a = np.matmul(T,a)
    b = np.matmul(T,b)
    c = np.matmul(T,c)
    d = np.matmul(T,d)
    e = np.matmul(T,e)
    f = np.matmul(T,f)
    g = np.matmul(T,g)
    i = np.matmul(T,i)
    j = np.matmul(T,j)
    k = np.matmul(T,k)
    l = np.matmul(T,l)

    plt.plot([a[0],c[0]],[a[1],c[1]],'g')
    plt.plot([a[0],b[0]],[a[1],b[1]],'g')
    plt.plot([d[0],c[0]],[d[1],c[1]],'g')
    plt.plot([g[0],b[0]],[g[1],b[1]],'g')
    plt.plot([d[0],g[0]],[d[1],g[1]],'g')
    plt.plot([d[0],i[0]],[d[1],i[1]],'g')
    plt.plot([g[0],l[0]],[g[1],l[1]],'g')
    plt.plot([k[0],f[0]],[k[1],f[1]],'g')
    plt.plot([j[0],e[0]],[j[1],e[1]],'g')
    plt.plot([j[0],i[0]],[j[1],i[1]],'g')
    plt.plot([k[0],l[0]],[k[1],l[1]],'g')
    plt.plot([x,a[0]],[y,a[1]],'k--')
    plt.plot(x,y,'mo')

    return a

def draw_pallet_jack(x,y,theta):
    pass

def transform_mat(x,y,theta):
    T = np.array([[np.cos(theta),-np.sin(theta),x],
                  [np.sin(theta),np.cos(theta),y],
                  [0,0,1]])
    return T

def calc_rear(x,y,theta,L):
    xr = x - L*math.cos(theta)
    yr = y - L*math.sin(theta)
    return(xr,yr)

def break_logic(xp,yp,xg,yg,x,y,x_rear,y_rear):
    slope1 = math.atan2((y-y_rear),(x-x_rear))
    slope2 = math.atan2((yg-y),(xg-x))
    print("slope1: {}, slope2: {}".format(slope1,slope2))
    if (abs(slope1 - slope2)<0.1 or abs(math.pi - abs(slope1 - slope2))<0.1) and math.sqrt((xp-x_rear)**2 + (yp-y_rear)**2)<0.1:
        return True
    else:
        return False

def calc_sign(x,y,theta,x_rear,y_rear,goal):
    angle1 = math.atan2((y-goal[1]),(x - goal[0]))
    angle2 = goal[2]
    front_back_indicator =  abs(wrapToPi(abs(angle1-angle2)))
    #

    # the difference between the two angle is just an
    # indicator of whether the point is in front or behind the goal
    # next thing we need is whether its facing towards the goal or away from the goal
    ang1 = theta #math.atan2((y-y_rear),(x-x_rear))
    ang2 = goal[2]
    towards_away_indicator = wrapToPi(abs(ang1-ang2))
    enablePrint()
    # print("Angle1 between front and rear")
    # print("Angle2 between goal and rear")
    # print(angle1,angle2)


    # print("front_back_indicator {}".format(math.degrees(front_back_indicator)))
    blockPrint()
    thresh = math.radians(89)

    if front_back_indicator < thresh: #and towards_away_indicator < 0.3:
        # its in front and facing towards the goal
        return -1
    elif front_back_indicator > thresh: #and towards_away_indicator < 0.3:
        # its in the back and still facing the goal
        return 1

def calc_pts(goal):
    xg = goal[0]
    yg = goal[1]
    thetag = goal[2]

    x1 = xg - 2*math.cos(thetag)
    y1 = yg - 2*math.sin(thetag)

    x2 = xg + 2*math.cos(thetag)
    y2 = yg + 2*math.sin(thetag)

    pt1 = [x1,y1]
    pt2 = [x2,y2]
    return pt1,pt2

def pure_pursuit():

    enablePrint()
    print("\n ##################################################################### \n")
    global DISABLE_SIGN, ROTATE_TO_ANGLE
    ld = 5
    # goal = [4,2,math.pi/3]
    goal = [random.uniform(0,10),random.uniform(0,10),random.uniform(-math.pi,math.pi)]
    pt1, pt2 = calc_pts(goal)# """ THIS IS WHERE THE LOGIC OF GENERATING WAY POINTS WOULD GO"""

    start = [random.uniform(0,10),random.uniform(0,10),random.uniform(-math.pi,math.pi)]
    # start = [1,2,math.pi-math.pi/3]
    v = 10
    s = 0
    L = 2.33 #base length
    vmax = 0.325

    x = start[0]
    y = start[1]
    theta = start[2]
    x_traj = []
    y_traj = []
    x_front = []
    y_front = []
    iter = 0
    sign = 1

    blockPrint()
    dec = False

    #decide on whether the ROTATE_TO_ANGLE should work or not
    RTA_decider = abs(wrapToPi(theta - goal[2]))
    enablePrint()
    if RTA_decider > math.pi/2:
        ROTATE_TO_ANGLE = True
        print("[INFO] ROTATE_TO_ANGLE required!")
    else:
        ROTATE_TO_ANGLE =  False
        print("[INFO] ROTATE_TO_ANGLE not required!")

    blockPrint()

    if ROTATE_TO_ANGLE:
        enablePrint()
        print("[INFO] Initiating ROTATE_TO_ANGLE Sequence")
        blockPrint()
        while True:
            plt.clf()
            plot_setup(x,y)
            x_traj.append(x)
            y_traj.append(y)

            # find the cordinates of the rear
            x_rear,y_rear = calc_rear(x,y,theta,L)

            xp,yp,xg,yg,ld= closest_pt(pt1,pt2,x_rear,y_rear,ld,dec)

            #find the value of beta
            beta = math.atan2((yg-y_rear),(xg-x_rear))
            #find alpha
            alpha = beta - theta

            line_theta = math.atan2(goal[1]-y,goal[0]-x)
            decision_angle = wrapToPi(theta - line_theta)

            # v should be calculated based on angle between goal and actual theta
            RTA_angle = wrapToPi(theta - goal[2]) # ROTATE_TO_ANGLE decision angle
            abs_RTA_angle = abs(RTA_angle)
            # v = 0.5*abs_RTA_angle
            v = vmax

            # v = 3
            # ld = 0.5*v #+  #2 0.5*math.sqrt((x-goal[0])**2 + (y-goal[1])**2)

            if v>vmax:
                v = vmax
            if v<-vmax:
                v = -vmax


            # if RTA_angle is positive then steering should be -90 else it should be +90
            if RTA_angle > 0 :
                s = -math.pi/2
            else:
                s = math.pi/2

            x,y,theta = update(x,y,theta,v,s)
            # a = draw_vehicle(x,y,theta)
            # x_front.append(a[0])
            # y_front.append(a[1])
            dpj(x,y,theta,s)
            plt.plot(x_traj,y_traj,'m--')
            plt.plot([x,xg],[y,yg],'k--')
            plt.plot([x,xp],[y,yp],'g--')
            plt.plot(xp,yp,'ro')
            plt.plot(xg,yg,'bo')
            plt.plot(x,y,'co')
            plot_theta(start,goal)
            plt.pause(PAUSE)
            if abs_RTA_angle<math.radians(60):
                v = 0
                print("[INFO] Goal Reached")
                enablePrint()
                print("[INFO] Exiting ROTATE_TO_ANGLE Sequence")
                blockPrint()
                break
            iter+=1
            if iter>500:
                v = 0
                print("[INFO] Goal not reached Forcing v = 0")
                enablePrint()
                print("[INFO] Exiting ROTATE_TO_ANGLE Sequence")
                blockPrint()
                break

        sign = calc_sign(x,y,theta,x_rear,y_rear,goal)


    dec =  False
    if ALIGN_TO_LINE:
        enablePrint()
        print("[INFO] Initiating ALIGN_TO_LINE Sequence")
        blockPrint()
        # one time decider for dec
        dec = one_time_password(x,y,theta,goal)
        if dec == False:
            sign = 1
        else:
            sign = -1
        enablePrint()
        print("[INFO] ONE TIME PASSWORD DEC: {}".format(dec))
        blockPrint()
        while True:
            plt.clf()
            plot_setup(x,y)
            x_traj.append(x)
            y_traj.append(y)

            # find the cordinates of the rear
            x_rear,y_rear = calc_rear(x,y,theta,L)

            xp,yp,xg,yg,ld= closest_pt(pt1,pt2,x_rear,y_rear,ld,dec)

            #find the value of beta
            beta = math.atan2((yg-y_rear),(xg-x_rear))
            #find alpha
            alpha = beta - theta

            line_theta = math.atan2(goal[1]-y,goal[0]-x)
            decision_angle = wrapToPi(theta - line_theta)

            # sign = 1
            # v = sign*1.5
            v = 0.4*sign*math.sqrt((x-goal[0])**2 + (y-goal[1])**2)

            # v = 3
            # ld = 0.5*v #+  #2 0.5*math.sqrt((x-goal[0])**2 + (y-goal[1])**2)

            if v>vmax:
                v = vmax
            if v<-vmax:
                v = -vmax

            s = math.atan2(2*L*math.sin(alpha),ld)

            if s>math.pi/2:
                s = math.pi/2
            if s<-math.pi/2:
                s = -math.pi/2
            # if decision_angle > math.pi/2 or decision_angle < -math.pi/2:
            #     sign = 1
            # else:
            #     sign = -1
                # s = -s
            enablePrint()
            # print("v: {} s: {}".format(v,math.degrees(s)))
            blockPrint()
            x,y,theta = update(x,y,theta,v,s)
            # a = draw_vehicle(x,y,theta)
            # x_front.append(a[0])
            # y_front.append(a[1])
            dpj(x,y,theta,s)
            plt.plot(x_traj,y_traj,'m--')
            # plt.plot(x_front,y_front,'b--')
            plt.plot([x,xg],[y,yg],'k--')
            plt.plot([x,xp],[y,yp],'g--')
            plt.plot(xp,yp,'ro')
            plt.plot(xg,yg,'bo')
            plt.plot(x,y,'co')
            plot_theta(start,goal)
            plt.pause(PAUSE)
            if break_logic(xp,yp,xg,yg,x,y,x_rear,y_rear):
                v = 0
                print("[INFO] Goal Reached")
                enablePrint()
                print("[INFO] Exiting ALIGN_TO_LINE Sequence")
                blockPrint()
                break
            iter+=1
            if iter>500:
                print("[INFO] Goal not reached")
                enablePrint()
                print("[INFO] Exiting ALIGN_TO_LINE Sequence")
                blockPrint()
                break

        sign = calc_sign(x,y,theta,x_rear,y_rear,goal)


    if TRANSLATE_TO_GOAL:
        while True:

            dist = sign*math.sqrt((x-goal[0])**2 + (y-goal[1])**2)

            plt.clf()
            plot_setup(x,y)
            x_traj.append(x)
            y_traj.append(y)

            # find the cordinates of the rear
            x_rear,y_rear = calc_rear(x,y,theta,L)
            v = dist*0.5
            s = 0
            x,y,theta = update(x,y,theta,v,s)

            # draw_vehicle(x,y,theta)
            # x_front.append(a[0])
            # y_front.append(a[1])
            dpj(x,y,theta,s)
            plt.plot(x_traj,y_traj,'m--')
            plt.plot([pt1[0],pt2[0]],[pt1[1],pt2[1]],'b',label='path')
            print(v)


            if abs(dist)<0.2:
                v = 0
                print("Goal Reached Hurray!")
                break
            plt.pause(PAUSE)

    if sign > 0:
        dec = False
    else:
        dec = True

    if MOVE_TO_GOAL:
        enablePrint()
        print("[INFO] Initiating MOVE_TO_GOAL Sequence")
        blockPrint()
        iter = 0
        while True:
            if not DISABLE_SIGN:
                if iter%10 == 0:
                    sign = calc_sign(x,y,theta,x_rear,y_rear,goal)
                    if sign > 0:
                        dec = False
                    else:
                        dec = True

            plt.clf()
            plot_setup(x,y)
            x_traj.append(x)
            y_traj.append(y)

            # find the cordinates of the rear
            x_rear,y_rear = calc_rear(x,y,theta,L)

            xp,yp,xg,yg,ld= closest_pt(pt1,pt2,x_rear,y_rear,ld,dec)

            #find the value of beta
            beta = math.atan2((yg-y_rear),(xg-x_rear))
            #find alpha
            alpha = beta - theta

            line_theta = math.atan2(goal[1]-y,goal[0]-x)
            decision_angle = wrapToPi(theta - line_theta)

            v = 0.4*sign*math.sqrt((x-goal[0])**2 + (y-goal[1])**2)

            # v = 3
            # ld = math.sqrt((x-goal[0])**2 + (y-goal[1])**2) #  + 0.5*v
            if v>vmax:
                v = vmax
            if v<-vmax:
                v = -vmax

            s = math.atan2(2*L*math.sin(alpha),ld)

            if s>math.pi/2:
                s = math.pi/2
            if s<-math.pi/2:
                s = -math.pi/2
            # if decision_angle > math.pi/2 or decision_angle < -math.pi/2:
            #     sign = 1
            # else:
            #     sign = -1
                # s = -s
            enablePrint()
            # print("v: {} s: {}".format(v,math.degrees(s)))
            blockPrint()
            x,y,theta = update(x,y,theta,v,s)
            # a = draw_vehicle(x,y,theta)
            # x_front.append(a[0])
            # y_front.append(a[1])
            dpj(x,y,theta,s)
            plt.plot(x_traj,y_traj,'m--')
            # plt.plot(x_front,y_front,'b--')
            plt.plot([x,xg],[y,yg],'k--')
            plt.plot([x,xp],[y,yp],'g--')
            plt.plot(xp,yp,'ro')
            plt.plot(xg,yg,'bo')
            plt.plot(x,y,'co')
            plt.plot(x_rear,y_rear,'ko')
            plot_theta(start,goal)
            plt.pause(PAUSE)
            if math.sqrt((x-goal[0])**2 + (y-goal[1])**2)<0.1:
                v = 0
                print("[INFO] Goal Reached")
                enablePrint()
                print("[INFO] Exiting MOVE_TO_GOAL Sequence")
                blockPrint()
                break
            iter+=1
            if iter>500:
                print("[INFO] Goal not reached")
                enablePrint()
                print("[INFO] Exiting MOVE_TO_GOAL Sequence")
                blockPrint()
                break

def motion_demo():
    v =1
    s = 0.5
    x = 5
    y = 2
    theta = math.pi/6
    x_traj = []
    y_traj = []
    for i in range(500):
        plt.cla()
        plot_setup()
        # draw_vehicle(x,y,theta)
        dpj(x,y,theta,v,s)
        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'k--')
        x,y,theta = update(x,y,theta,v,s)
        v = random.uniform(0,2)
        s = random.uniform(0,math.pi/6)
        plt.pause(PAUSE)

if __name__ == "__main__":
    try:
        for i in range(5):
            pure_pursuit()
        # pure_pursuit()
        enablePrint()
        print("[INFO] Task Finshed")
        print("\n **************************************************************\n")
        plt.show()
    except KeyboardInterrupt:
        pass
