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
ROTATE_AT_POINT = False

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

def plot_goals(goals):
    for goal in goals:
        plt.arrow(goal[0], goal[1], 1.5*m.cos(goal[2]), 1.5*m.sin(goal[2]),
          head_width=0.1, head_length=0.2)

def seek_one_pure_pursuit(start,goal,true_goal,perp):

    signal = False
    xs,ys,thetas = start
    xt, yt = goal #this is the target point
    xg, yg, thetag = true_goal #this is the final goal

    """
    EXPERIMENTAL FAILED
    """
    # shift_angle = m.atan2((perp[1] - yt), (perp[0]-xt))
    # xt = perp[0] - (2.33 + 0.85)*m.cos(shift_angle)
    # yt = perp[1] - (2.33 + 0.85)*m.sin(shift_angle)
    """
    NEED TO SEE IF THE Ld should be changed
    that will happen when the angle check_angle is more than 85 here
    MAYBE NOT -EXPERIMENTAL
    """
    phi = m.atan2((yt-ys),(xt-xs))
    heading_error = wrapToPi(phi - thetas)
    if heading_error > (m.pi/2-m.radians(1))  or heading_error <-(m.pi/2-m.radians(1)):
        signal = True
    #     shift_angle = m.atan2((perp[1] - yt), (perp[0]-xt))
    #     xt = perp[0] - (0.85)*m.cos(shift_angle)
    #     yt = perp[1] - (0.85)*m.sin(shift_angle)
    #     plt.plot(xt,yt,'bo')
    #     print("*******************REVERSING***********************************")
    """
    LONGITUDINAL CONTROL USING PID
    """

    # Kpd = 0.5
    # v = Kpd*m.sqrt((xs-true_goal[0])**2+ (ys-true_goal[1])**2)
    # if v>0.325:
    #     v = 0.325
    # if v<-0.325:
    #     v = -0.325

    """
    FULL SPEED CONTROL
    """
    v = 1.325
    """
    LONGITUDINAL CONTROL USING CURVATURE BASED LAW
    """
    # add = 1/(ld/(2*m.sin(alpha+0.00001)))
    # scale_factor = 1.5
    # v = scale_factor*abs(0.325/(0.325 + add ))
    # if v>0.325:
    #     v = 0.325
    # if v<-0.325:
    #     v = -0.325
    """
    LATERAL CONTROL
    """
    ld = m.sqrt((xs - xt)**2 + (ys - yt)**2) #this is the lookahead distance
    #need to find the angle alpha
    beta = m.atan2((yt-ys),(xt - xs))
    alpha = wrapToPi(beta - thetas)
    s = -m.atan2((2*2.33*m.sin(alpha)), (ld)) #since we are travelling in reverse the steering angle would have to be originally negative



    """
    decision on front or back based on heading_error signal
    """
    if signal == True:
        v = -v


    xs,ys,thetas = update_rear(xs,ys,thetas,v,s)
    return xs,ys,thetas,s

def calc_perp(x,y,theta,pt1,pt2):
    """
    ALL OF A SUDDEN THIS BECOMES AN IMPORTANT LOGIC
    """
    #this also needs to tell me if the xt,yt point has gone beyond pt2
    skip = False
    ld = 0.45
    INF_ERROR = False
    # ld = 0.85 #this worked best
    # ld = 5
    #find the equation of the line
    x1,y1 = pt1
    x2,y2 = pt2
    try:
        slope = (y2-y1)/(x2-x1)
    except:
        INF_ERROR = True

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
        phi = m.atan2((y2-y1),(x2-x1))
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
            points_considered = [pt1, pt2]
            final_perp_dist = perp_dist
    #CALCULATE PHI AND BETA
    xt, yt = best_target
    beta = m.atan2((yt-y),(xt-x))
    pt1, pt2 = points_considered
    phi = m.atan2((pt2[1]-pt1[1]),(pt2[0]-pt1[0]))
    return (best_target,proximity,skip,[xp,yp],phi,beta)

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

def seek_one_pid(start,goal,true_goal,perp,phi,beta):
    """
    PROCESSING INCOMING DATA
    """
    x,y,theta = start
    xp, yp = perp
    #CROSS TRACK ERROR
    cross_track_error = m.sqrt((x-xp)**2 + (y-yp)**2)

    #INDICATION ON WHICH SIDE IS THE VEHICLE FROM THE PATH
    indicator = True
    xt, yt = goal
    beta = m.atan2((yt-y),(xt-x))
    if (beta - phi)<0:
        indicator = False

    #raw heading error
    heading_error = wrapToPi(beta - theta)
    signal= False
    # print(m.degrees(heading_error),m.degrees(beta))
    if heading_error >= m.pi/2:
        heading_error = (m.pi - heading_error)
        signal = True
    if heading_error <= -m.pi/2:
        heading_error = -(m.pi - abs(heading_error))
        signal = True
    """
    LONGITUDINAL CONTROL
    """
    v = 0.325
    if signal == True:
        v = -v

    """
    LATERAL CONTROL
    """
    Kps = 3
    if indicator:
        s = -Kps*cross_track_error
    else:
        s = Kps*cross_track_error

    if s > m.pi/2:
        s = m.pi/2
    if s<-m.pi/2:
        s = -m.pi/2

    x,y,theta = update_rear(x,y,theta,v,s)
    return x,y,theta,s

def path_track4(path,thetas,x_lim,y_lim, x_traj_tot, y_traj_tot,goals):
    # thetas = 0 #cancelling user defined theta
    win_zoom = 7
    """
    SAMPLING STARTS HERE
    """
    final_path = []
    x,y = path[0]
    sample_rate = 1 #best was 2 #changed again from 0.2
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
    dist_thresh = 0.1#0.05
    while m.sqrt((x - goal[0])**2 + (y - goal[1])**2)>dist_thresh:
        #first step would be to find the target point
        target,proximity,skip,perp,phi,beta = calc_target(x,y,theta,dummy_gp)
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
            target, proximity, skip, perp, phi, beta = calc_target(x,y,theta,dummy_gp)
            print(skip)
        plt.cla()
        plt.axis('scaled')
        plt.xlim(x-win_zoom,x+win_zoom)
        plt.ylim(y-win_zoom,y+win_zoom)
        plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
        plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
        plt.plot(x_traj_tot, y_traj_tot, 'g--',label="REPLANNED PATH")
        plot_goals(goals)
        plt.plot(start[0],start[1],'co')
        plt.plot(xt,yt,'ro')

        if DRAW_DIFF:
            draw_robot(x,y,theta)
        if DRAW_PALLET:
            dpj(x,y,theta,s)

        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
        # x,y,theta,s = seek_one_pid([x,y,theta],[xt,yt],goal,perp,phi,beta)
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
            plt.xlim(x-win_zoom,x+win_zoom)
            plt.ylim(y-win_zoom,y+win_zoom)
            plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
            plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
            plt.plot(x_traj_tot, y_traj_tot, 'g--',label="REPLANNED PATH")
            plot_goals(goals)
            plt.plot(start[0],start[1],'co')
            plt.plot(xt,yt,'ro')

            if DRAW_DIFF:
                draw_robot(x,y,theta)
            if DRAW_PALLET:
                dpj(x,y,theta,s)

            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
            x,y,theta,s = seek_one_pid([x,y,theta],[xt,yt],goal,perp,phi,beta)
            # print(m.degrees(s))
            plt.pause(0.0001)
    if ROTATE_AT_POINT:
        """
        This will come into picture when the final goal orienation has been met
        """
        #first need to decide whether to rotate clockwise or counter clockwise
        Kpv = 0.3
        heading_error = wrapToPi(theta_g + theta)
        while abs(heading_error) > 0.05:
            plt.cla()
            plt.axis('scaled')
            plt.xlim(x-win_zoom,x+win_zoom)
            plt.ylim(y-win_zoom,y+win_zoom)
            plt.plot(gp_array[:,0],gp_array[:,1],'m',label="Sampled-Target-path")
            plt.plot(prev_path_array[:,0],prev_path_array[:,1],'c--',label="Actual-Target-path")
            plt.plot(x_traj_tot, y_traj_tot, 'g--',label="REPLANNED PATH")
            plot_goals(goals)
            plt.plot(start[0],start[1],'co')
            v = Kpv*heading_error

            if v>0.325:
                v = 0.325
            if v<-0.325:
                v = -0.325

            s = m.pi/2
            x,y,theta  = update_rear(x,y,theta,v,s)
            heading_error = wrapToPi(theta_g + theta)
            if DRAW_DIFF:
                draw_robot(x,y,theta)
            if DRAW_PALLET:
                dpj(x,y,theta,s)
            x_traj.append(x)
            y_traj.append(y)
            plt.plot(x_traj,y_traj,'r',label="Actual-Path-taken")
            plt.pause(0.0001)


    print("Time taken: {} s".format(time.time()-tic))
    # plt.title('PID BASED CONSTANT SPEED PATH TRACKING OF A PALLET JACK')
    plt.title('PURE-PURSUIT BASED PATH TRACKING OF A PALLET JACK')
    plt.legend()
    # plt.show()
    return x,y,theta
