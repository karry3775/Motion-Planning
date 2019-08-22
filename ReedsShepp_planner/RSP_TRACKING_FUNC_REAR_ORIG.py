from __future__ import division
import matplotlib.pyplot as plt
import math as m
import numpy as np
from draw_diff_drive import draw_robot
# from drawing_pallet_jack import dpj
from drawing_pallet_jack_rev import dpj
import copy
import time

DRAW_PALLET = True
DRAW_DIFF = False
ALIGN_TO_GOAL_LINE = False

def update(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x = x + v*m.cos(theta+s)*dt
    y = y + v*m.sin(theta+s)*dt
    theta = wrapToPi(theta + (v/L)*m.sin(s)*dt)
    return(x,y,theta)

def update_rear(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x+= (v/2)*(m.cos(theta-s) + m.cos(theta+s))*dt
    y+= (v/2)*(m.sin(theta-s) + m.sin(theta+s))*dt
    theta+= -(v/L)*m.sin(s)*dt
    return(x,y,theta)

def wrapToPi(theta):
        return m.atan2(m.sin(theta),m.cos(theta))

def calc_perp(x,y,theta,pt1,pt2):
    #this also needs to tell me if the xt,yt point has gone beyond pt2
    skip = False
    ld = 0.85
    INF_ERROR = False
    # ld = 0.85 #this worked best
    # ld = 5
    #find the equation of the line
    x1,y1 = pt1
    x2,y2 = pt2
    if not INF_ERROR:
        print("[NO ERROR]")
        phi = m.atan2((y2-y1),(x2-x1))
        slope = (y2-y1)/(x2-x1)
        if abs(slope) == float("inf"):
            INF_ERROR = True
        print(slope)
        intercept = y1 - slope*x1
        xp = (x + slope*y - slope*intercept)/(1+slope**2)
        yp = intercept + slope*xp
        xt = xp + m.cos(phi)*ld
        yt = yp + m.sin(phi)*ld
        #checking if it has crossed pt2
        check_angle = m.atan2((yt-y2),(xt-x2))
        if check_angle == phi:
            skip = True
            xt = x2
            yt = y2
        #checking if the point is behind pt1
        """
        MEASURES TO PREVENT WEIRD BEHAVIOUR
        """
        check_angle2 = m.atan2((y1-yt),(x1-xt))
        if check_angle2 == phi:
            xt = x1 + m.cos(phi)*0.1
            yt = y1 + m.sin(phi)*0.1
        # elif abs(wrapToPi(phi-theta))>m.radians(80):  #this means that the target point has cleared the pt1
        #     xt = x1 + m.cos(phi)*4
        #     yt = y1 + m.sin(phi)*4
        # else:
        #     pass

    if INF_ERROR:
        print("[ZERO DIV ERROR]")
        xp = x1
        yp = y
        sign = (y2-y1)/abs(y2-y1)
        xt = xp
        yt = yp + sign*1
        #here also we need to check the crossing thresh
        if sign>0:
            if yt>y2:
                skip = True
            if yt<y1:
                yt = y1
                xt = x2
                yt = y2
        elif yt<y2:
            skip = True
            if yt>y1:
                yt = y1
            xt = x2
            yt = y2
        """
        MEASURES TO PREVENT WEIRD BEHAVIOUR
        """
        check_angle2 = m.atan2((y1-yt),(x1-xt))
        if check_angle2 == phi:
            xt = x1 + m.cos(phi)*0.1
            yt = y1 + m.sin(phi)*0.1

    return(xp,yp,xt,yt,skip)

def calc_target(x,y,theta,goal_points):
    if len(goal_points) >=3:
        goal_points = goal_points[0:3]
    else:
        pass
    best_dist = 999 #random
    best_pt = [x,y] #random
    best_target = [x,y] #random
    proximity = 999 #random
    skip = False
    for i in range(len(goal_points)-1):
        pt1 = goal_points[i]
        pt2 = goal_points[i+1]
        xp,yp,xt,yt,skip = calc_perp(x,y,theta,pt1,pt2)
        perp_dist = m.sqrt((x-xp)**2 + (y-yp)**2)
        if perp_dist < best_dist:
            best_dist = perp_dist
            best_pt = goal_points[i]
            next_pt = goal_points[i+1]
            best_target[0] =xt
            best_target[1] =yt
            proximity = m.sqrt((next_pt[0]-xt)**2 + (next_pt[1]-yt)**2) #to pt2

    return (best_target,proximity,skip)

def seek_one(start,goal,true_goal):
    x,y,theta = start
    Kp = 2
    # Kp = 0.85
    Kpd = 0.5
    v = Kpd*m.sqrt((x-true_goal[0])**2+ (y-true_goal[1])**2)
    if v>0.325:
        v = 0.325
    if v<-0.325:
        v = -0.325
    #thets calculations
    beta = m.atan2((goal[1]-y),(goal[0]-x))
    dist_error = m.sqrt((x-goal[0])**2 + (y-goal[1])**2)
    heading_error = wrapToPi(beta - theta)
    signal= False
    # print(m.degrees(heading_error),m.degrees(beta))
    if heading_error >= m.pi/2:
        heading_error = (m.pi - heading_error)
        signal = True
    if heading_error <= -m.pi/2:
        heading_error = -(m.pi - abs(heading_error))
        signal = True
    s = -Kp*heading_error

    if s > m.pi/2:
        s = m.pi/2
    if s<-m.pi/2:
        s = -m.pi/2
    if signal == True:
        v = -v
        # v = steering_sign*v
        # s = -s
    #

    x,y,theta = update_rear(x,y,theta,v,s)
    return x,y,theta,s

def path_track(path):
    xstart, ystart = path[0]
    start = [xstart,ystart,0]
    goal_points = path
    dummy_gp = copy.deepcopy(goal_points)
    #need to calculate goal theta last two points
    last_pt = dummy_gp[-1]
    second_last_pt = dummy_gp[-2]
    theta_g = m.atan2((last_pt[1]-second_last_pt[1]),(last_pt[0]-second_last_pt[0]))
    goalx,goaly = path[-1]
    goal = [goalx,goaly,theta_g]
    # goal_points = [[2,2]]
    x,y,theta = start
    v = 1
    s = 0
    gp_array = np.array(goal_points)
    x_traj = []
    y_traj = []


    skip = False
    while len(dummy_gp) >1:
        #first step would be to find the target point
        target,proximity,skip = calc_target(x,y,theta,dummy_gp)
        xt,yt = target
        if proximity<0.1 or skip==True:
            dummy_gp.pop(0)
        if skip==True:
            print(skip)
        plt.cla()
        plt.axis('scaled')
        plt.xlim(-10,15)
        plt.ylim(-10,15)
        plt.plot(gp_array[:,0],gp_array[:,1],'--')
        plt.plot(start[0],start[1],'co')
        plt.plot(xt,yt,'ro')

        if DRAW_DIFF:
            draw_robot(x,y,theta)
        if DRAW_PALLET:
            dpj(x,y,theta,s)

        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'r')
        x,y,theta,s = seek_one([x,y,theta],[xt,yt],goal)
        # print(m.degrees(s))
        plt.pause(0.0001)
    if ALIGN_TO_GOAL_LINE:
        pt1 = goal_points[-2]
        pt2 = goal_points[-1]
        while wrapToPi(abs(theta - goal[2]))>0.1:
            _,_,xt,yt,_ = calc_perp(x,y,pt1,pt2)
            plt.cla()
            plt.axis('scaled')
            plt.xlim(-5,15)
            plt.ylim(-5,15)
            plt.plot(gp_array[:,0],gp_array[:,1],'--')
            plt.plot(start[0],start[1],'co')
            plt.plot(xt,yt,'ro')

            if DRAW_DIFF:
                draw_robot(x,y,theta)
            if DRAW_PALLET:
                dpj(x,y,theta,s)

            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r')
            x,y,theta,s = seek_one([x,y,theta],[xt,yt],goal)
            # print(m.degrees(s))
            plt.pause(0.0001)
    # plt.show()

def path_track2(path):
    """
    SAMPLING STARTS HERE
    """
    final_path = []
    x,y = path[0]
    sample_rate = 2
    final_path.append([x,y])
    for x,y in path:
        xf,yf = final_path[-1]
        if m.sqrt((xf-x)**2 + (yf-y)**2)>sample_rate:
            final_path.append([x,y])
        else:
            continue

    """
    SAMPLING FINISHES HERE
    """

    prev_path = path
    path = final_path
    prev_path_array = np.array(prev_path)

    tic = time.time()
    xstart, ystart = path[0]
    start = [xstart,ystart,0]
    goal_points = path

    dummy_gp = copy.deepcopy(goal_points)


    #need to calculate goal theta last two points
    last_pt = dummy_gp[-1]
    second_last_pt = dummy_gp[-2]
    theta_g = m.atan2((last_pt[1]-second_last_pt[1]),(last_pt[0]-second_last_pt[0]))
    goalx,goaly = goal_points[-1]
    goal = [goalx,goaly,theta_g]

    x,y,theta = start
    v = 1
    s = 0
    gp_array = np.array(goal_points)
    x_traj = []
    y_traj = []

    skip = False
    while len(dummy_gp) >1:
        #first step would be to find the target point
        target,proximity,skip = calc_target(x,y,theta,dummy_gp)
        xt,yt = target
        if proximity<0.3 or skip==True:
            dummy_gp.pop(0)
        if skip==True:
            #need to set the value of target to something
            target, proximity, skip = calc_target(x,y,theta,dummy_gp)
            print(skip)
        plt.cla()
        plt.axis('scaled')
        plt.xlim(-10,15)
        plt.ylim(-10,15)
        plt.plot(gp_array[:,0],gp_array[:,1],'m--',label="Sampled-Target-path")
        plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
        plt.plot(start[0],start[1],'co')
        plt.plot(xt,yt,'ro')

        if DRAW_DIFF:
            draw_robot(x,y,theta)
        if DRAW_PALLET:
            dpj(x,y,theta,s)

        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
        x,y,theta,s = seek_one([x,y,theta],[xt,yt],goal)
        # print(m.degrees(s))
        plt.pause(0.0001)
    if ALIGN_TO_GOAL_LINE:
        pt1 = goal_points[-2]
        pt2 = goal_points[-1]
        while wrapToPi(abs(theta - goal[2]))>0.1:
            _,_,xt,yt,_ = calc_perp(x,y,theta,pt1,pt2)
            plt.cla()
            plt.axis('scaled')
            plt.xlim(-10,15)
            plt.ylim(-10,15)
            plt.plot(gp_array[:,0],gp_array[:,1],'m--',label="Sampled-Target-path")
            plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
            plt.plot(start[0],start[1],'co')
            plt.plot(xt,yt,'ro')

            if DRAW_DIFF:
                draw_robot(x,y,theta)
            if DRAW_PALLET:
                dpj(x,y,theta,s)

            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
            x,y,theta,s = seek_one([x,y,theta],[xt,yt],goal)
            # print(m.degrees(s))
            plt.pause(0.0001)
    print("Time taken: {} s".format(time.time()-tic))
    plt.title('PID BASED CONSTANT SPEED PATH TRACKING OF A PALLET JACK')
    plt.legend()
    plt.show()

def path_track3(path,thetas):
    # thetas = 0 #cancelling user defined theta
    """
    SAMPLING STARTS HERE
    """
    final_path = []
    x,y = path[0]
    sample_rate = 0.2 #best was 2
    final_path.append([x,y])
    for x,y in path:
        xf,yf = final_path[-1]
        if m.sqrt((xf-x)**2 + (yf-y)**2)>sample_rate:
            final_path.append([x,y])
        else:
            continue

    if path[-1] not in final_path:
        final_path.append(path[-1])


    """
    SAMPLING FINISHES HERE
    """

    prev_path = path
    path = final_path
    prev_path_array = np.array(prev_path)

    tic = time.time()
    xstart, ystart = path[0]
    start = [xstart,ystart,thetas]
    goal_points = path

    dummy_gp = copy.deepcopy(goal_points)


    #need to calculate goal theta last two points
    last_pt = dummy_gp[-1]
    second_last_pt = dummy_gp[-2]
    theta_g = -m.atan2((last_pt[1]-second_last_pt[1]),(last_pt[0]-second_last_pt[0]))
    goalx,goaly = goal_points[-1]
    goal = [goalx,goaly,theta_g]

    x,y,theta = start
    v = 1
    s = 0
    gp_array = np.array(goal_points)
    x_traj = []
    y_traj = []

    skip = False
    while m.sqrt((x - goal[0])**2 + (y - goal[1])**2)>0.2:
        #first step would be to find the target point
        target,proximity,skip = calc_target(x,y,theta,dummy_gp)
        xt,yt = target
        if (proximity<0.1 or skip==True) and len(dummy_gp)>2:
            dummy_gp.pop(0)
        if len(dummy_gp)==2:
            print(dummy_gp)
            angle = theta_g
            xt = last_pt[0] + 0*m.cos(theta_g)
            yt = last_pt[1] + 0*m.sin(-theta_g)
            plt.plot(xt,yt,'ko')

        if skip==True:
            #need to set the value of target to something
            target, proximity, skip = calc_target(x,y,theta,dummy_gp)
            print(skip)
        plt.cla()
        plt.axis('scaled')
        plt.xlim(-10,15)
        plt.ylim(-10,15)
        plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
        plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
        plt.plot(start[0],start[1],'co')
        plt.plot(xt,yt,'ro')

        if DRAW_DIFF:
            draw_robot(x,y,theta)
        if DRAW_PALLET:
            dpj(x,y,theta,s)

        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
        x,y,theta,s = seek_one([x,y,theta],[xt,yt],goal)
        # print(m.degrees(s))
        plt.pause(0.0001)
    if ALIGN_TO_GOAL_LINE:
        pt1 = goal_points[-2]
        pt2 = goal_points[-1]
        while wrapToPi(abs(theta - goal[2]))>0.1:
            _,_,xt,yt,_ = calc_perp(x,y,theta,pt1,pt2)
            plt.cla()
            plt.axis('scaled')
            plt.xlim(-10,15)
            plt.ylim(-10,15)
            plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
            plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
            plt.plot(start[0],start[1],'co')
            plt.plot(xt,yt,'ro')

            if DRAW_DIFF:
                draw_robot(x,y,theta)
            if DRAW_PALLET:
                dpj(x,y,theta,s)

            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
            x,y,theta,s = seek_one([x,y,theta],[xt,yt],goal)
            # print(m.degrees(s))
            plt.pause(0.0001)
    print("Time taken: {} s".format(time.time()-tic))
    plt.title('PID BASED CONSTANT SPEED PATH TRACKING OF A PALLET JACK')
    plt.legend()
    # plt.show()
    return theta

"""
 DEMO CODE

tic = time.time()
plt.title('PID BASED PATH TRACKING OF A PALLET JACK')
# start = [2.5,10,0]
# goal_points = [[5,8],[10,5],[15,9],[20,1],[25,21],[1,20],[3,25]] #successfull path follow
# start = [0,0,0]
# goal_points = [[1,2],[2,4],[3,5],[4,7],[5,8]] # successfull path follow
# start = [-2,-2,0]
# goal_points = [[0,0],[1.98,2.375],[4.04,4.23],[6.28,5],[8.75,4.079],[10.51,2.45],[12,0],[14,-2]] #succesfull
# start = [0,0,0]
# goal_points = [[0,0],[2,2],[6,7],[9,10]] #succesfull
# start =[0,0,0]
# goal_points =[[0,0],[5,5],[7,0]]
# start = [-2,-2,0]
# goal_points = [[0,0],[1,2],[1,-5],[5,6]]
"""
# SQUARE DEMO
"""
start = [0,0,0]
goal_points = [[0,0],[10,0],[10,10],[0,10],[0,0],[10,0],[10,10],[0,10],[0,0],[10,0],[10,10],[0,10],[0,0]] #circular polygon follow
"""
# SPIRAL DEMO
"""
# start = [0,0,0]
# goal_points = [[0,0],[10,0],[10,8],[3,8],[3,2],[8,2],[8,6],[4,6],[4,3],[12,3]]

"""
# PENTAGON DEMO
"""
# start = [0,0,0]
# goal_points = [[0,0],[5,0],[9,4],[2.5,8],[-4,4],[0,0]]

"""
# Square then pentagon demo
"""
# start = [0,0,0]
# goal_points = [[0,0],[10,0],[10,10],[0,10],[0,0],[5,0],[9,4],[2.5,8],[-4,4],[0,0]]

"""
# Figure 8 demo
"""
# start = [0,0,0]
# goal_points = [[0,0],[5,0],[9,4],[9,6.5],[2.5,8],[-4,6.5],[-4,4],[0,0],[5,0],[9,-4],[9,-6.5],[2.5,-8],[-4,-6.5],[-4,-4],[0,0]]
"""
# SINE WAVE DEMO
"""
# start = [-2,-2,0]
# goal_points = [[0,0],[1.98,2.375],[4.04,4.23],[6.28,5],[8.75,4.079],[10.51,2.45],[12,0],[14,-2]]

"""
# PARALLEL PARK DEMO
"""
start = [10,10,m.pi/2]
goal_points = [[10,10],[6,6],[3,2],[3,0],[3,8],[3,9],[3,11],[3,10],[3,6],[1,2],[1,0]]
#find necessary theta
thetas = m.atan2((6 - 10),(6 - 10))
start[2] = thetas

dummy_gp = copy.deepcopy(goal_points)


#need to calculate goal theta last two points
last_pt = dummy_gp[-1]
second_last_pt = dummy_gp[-2]
theta_g = m.atan2((last_pt[1]-second_last_pt[1]),(last_pt[0]-second_last_pt[0]))
goalx,goaly = goal_points[-1]
goal = [goalx,goaly,theta_g]
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
    target,proximity,skip = calc_target(x,y,theta,dummy_gp)
    xt,yt = target
    if proximity<0.1 or skip==True:
        dummy_gp.pop(0)
    if skip==True:
        print(skip)
    plt.cla()
    plt.axis('scaled')
    plt.xlim(-10,15)
    plt.ylim(-10,15)
    plt.plot(gp_array[:,0],gp_array[:,1],'--',label="Target-path")
    plt.plot(start[0],start[1],'co')
    plt.plot(xt,yt,'ro')

    if DRAW_DIFF:
        draw_robot(x,y,theta)
    if DRAW_PALLET:
        dpj(x,y,theta,s)

    x_traj.append(x)
    y_traj.append(y)
    plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
    x,y,theta,s = seek_one([x,y,theta],[xt,yt])
    print(m.degrees(s))
    plt.pause(0.0001)
if ALIGN_TO_GOAL_LINE:
    pt1 = goal_points[-2]
    pt2 = goal_points[-1]
    while wrapToPi(abs(theta - goal[2]))>0.1:
        _,_,xt,yt,_ = calc_perp(x,y,theta,pt1,pt2)
        plt.cla()
        plt.axis('scaled')
        plt.xlim(-10,15)
        plt.ylim(-10,15)
        plt.plot(gp_array[:,0],gp_array[:,1],'--',label="Target-path")
        plt.plot(start[0],start[1],'co')
        plt.plot(xt,yt,'ro')

        if DRAW_DIFF:
            draw_robot(x,y,theta)
        if DRAW_PALLET:
            dpj(x,y,theta,s)

        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
        x,y,theta,s = seek_one([x,y,theta],[xt,yt])
        print(m.degrees(s))
        plt.pause(0.0001)
print("Time taken: {} s".format(time.time()-tic))
plt.title('PID BASED PATH TRACKING OF A PALLET JACK')
plt.legend()
plt.show()


#there might be a situation when the final orienation is not matching the intended orientation






"""


#make the speed explicitly 0 after this
