


from __future__ import division
import matplotlib.pyplot as plt
import math as m
import numpy as np
import random

"""
REFERENCES:
FOR INTERSECTING POINTS BETWEEN TWO CIRCLES:
https://stackoverflow.com/questions/3349125/circle-circle-intersection-points
DUBINS CURVE:
https://gieseanw.files.wordpress.com/2012/10/dubins.pdf
"""
def wrapToPi(theta):
    return(m.atan2(m.sin(theta),m.cos(theta)))

def fcp(pose):
    off = 2.33 #which is the minimum turning radius
    x = pose[0]
    y = pose[1]
    theta = pose[2]

    angle_L = wrapToPi(theta + m.pi/2)
    angle_R = wrapToPi(theta - m.pi/2)

    center_L = [x + off*m.cos(angle_L),y + off*m.sin(angle_L)]
    center_R = [x + off*m.cos(angle_R),y + off*m.sin(angle_R)]

    return(center_L,center_R)

def draw_circles(pose):
    center_L, center_R = fcp(pose)
    L1 = plt.Circle((center_L[0], center_L[1]), 2.33, color='m', fill=False)
    R1 = plt.Circle((center_R[0], center_R[1]), 2.33, color='b', fill=False)
    # plt.gca().add_patch(L1)
    # plt.gca().add_patch(R1)
    return(center_L,center_R)

def find_tangents(C1, C2, dir1, dir2):
    INNER_TANGENTS = True
    tangent_pts1 = []
    tangent_pts2 = []
    start = C1
    goal  = C2
    x1,y1 = start
    x2,y2 = goal
    r1 = 2.33
    r2 = 2.33
    #####################################
    # INNER TANGENTS
    #####################################
    #NEED TO CHECK IF INNER TANGENTS EXIST OR NOT
    print(m.sqrt((x1-x2)**2 + (y1-y2)**2))
    if m.sqrt((x1-x2)**2 + (y1-y2)**2) > (r1+r2):
        print("[INFO] Inner tangents exist")
        #find the circle centered between two points (Circle 3)
        x3 = (start[0] + goal[0])/2
        y3 = (start[1] + goal[1])/2
        r3 = m.sqrt((start[0]-goal[0])**2 + (start[1] - goal[1])**2)/2
        C3 = plt.Circle((x3, y3), r3, color='k', fill = False)
        # plt.gca().add_patch(C3)

        #find the circle centered centered at start and with radius r1 + r2
        x4 = start[0]
        y4 = start[1]
        r4 = 2.33*2
        C4 = plt.Circle((x4, y4), r4, color='g', fill = False)
        # plt.gca().add_patch(C4)

        #find the intersection between C3 and C4
        d = m.sqrt((x3-x4)**2 + (y3-y4)**2)
        a = (r4**2 - r3**2 + d**2)/(2*d)
        h = m.sqrt(r4**2 - a**2)
        ang = m.atan2((y3-y4),(x3-x4))
        x_m = x4 + a*m.cos(ang)
        y_m = y4 + a*m.sin(ang)
        ang_perp1 = wrapToPi(ang + m.pi/2)
        ang_perp2 = wrapToPi(ang - m.pi/2)
        x_int1 = x_m + h*m.cos(ang_perp1)
        y_int1 = y_m + h*m.sin(ang_perp1)
        x_int2 = x_m + h*m.cos(ang_perp2)
        y_int2 = y_m + h*m.sin(ang_perp2)

        # plt.plot(x_int1,y_int1,'co')
        # plt.plot(x_int2,y_int2,'co')
        """
        first intersection point calc
        """
        #find angle towards to the x_int1 and y_int1
        ang = m.atan2((y_int1-y4),(x_int1-x4))
        x_t1 = x4 + 2.33*m.cos(ang)
        y_t1 = y4 + 2.33*m.sin(ang)

        #find the inner point on the next circle C2
        mag = m.sqrt((x_int1-x2)**2 + (y_int1-y2)**2)
        dir = m.atan2((y2-y_int1),(x2-x_int1))
        x_t11 = x_t1 + mag*m.cos(dir)
        y_t11 = y_t1 + mag*m.sin(dir)

        """
        second intersection point calc
        """
        #find angle towards to the x_int2 and y_int2
        ang = m.atan2((y_int2-y4),(x_int2-x4))
        x_t2 = x4 + 2.33*m.cos(ang)
        y_t2 = y4 + 2.33*m.sin(ang)

        #find the inner point on the next circle C2
        mag = m.sqrt((x_int2-x2)**2 + (y_int2-y2)**2)
        dir = m.atan2((y2-y_int2),(x2-x_int2))
        x_t22 = x_t2 + mag*m.cos(dir)
        y_t22 = y_t2 + mag*m.sin(dir)


        if dir1 == "right" and dir2 == "left":
            # plt.plot([x4, x_t1],[y4, y_t1])
            tangent_pts1.append([x_t1,y_t1])
            # plt.plot([x_t1,x_t11],[y_t1,y_t11])
            tangent_pts2.append([x_t11,y_t11])
        if dir1 == "left" and dir2 == "right":
            # plt.plot([x4, x_t2],[y4, y_t2])
            tangent_pts1.append([x_t2, y_t2])
            # plt.plot([x_t2,x_t22],[y_t2,y_t22])
            tangent_pts2.append([x_t22, y_t22])


    else:
        print("[INFO] Inner tangents dont exist")
        INNER_TANGENTS = False

    #####################################
    # OUTER TANGENTS
    #####################################
    #since the circles are of same radius we can use simplications
    #begin by finding angle of the line conecting theie centers
    ang = m.atan2((y2-y1),(x2-x1))
    ang1 = wrapToPi(ang + m.pi/2)
    ang2 = wrapToPi(ang - m.pi/2)
    x_out11 = x1 + r1*m.cos(ang1)
    y_out11 = y1 + r1*m.sin(ang1)

    x_out12 = x1 + r1*m.cos(ang2)
    y_out12 = y1 + r1*m.sin(ang2)

    x_out21 = x2 + r2*m.cos(ang1)
    y_out21 = y2 + r2*m.sin(ang1)

    x_out22 = x2 + r2*m.cos(ang2)
    y_out22 = y2 + r2*m.sin(ang2)

    if (dir1 == "right" and dir2 == "right") or INNER_TANGENTS==False:
        tangent_pts1.append([x_out11, y_out11])
        tangent_pts2.append([x_out21, y_out21])
        # plt.plot([x_out11, x_out21],[y_out11, y_out21])
    if (dir1 == "left" and dir2 == "left") or INNER_TANGENTS==False:
        tangent_pts1 = []
        tangent_pts2 = []
        tangent_pts1.append([x_out12, y_out12])
        tangent_pts2.append([x_out22, y_out22])
        # plt.plot([x_out12, x_out22],[y_out12, y_out22])



    return(tangent_pts1, tangent_pts2)


def RSP(start,goal,dir1,dir2):
    # start = [0,0,0]
    # goal = [5,5,m.pi/2]
    #
    # start = [random.uniform(1,9),random.uniform(1,9),random.uniform(-m.pi/2,m.pi/2)]
    # goal = [random.uniform(1,9),random.uniform(1,9),random.uniform(-m.pi/2,m.pi/2)]

    # plt.axis('scaled')
    # plt.xlim(-20,20)
    # plt.ylim(-20,20)
    # plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
    # plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='r',head_width=0.5, head_length=1)

    #find the center point of the circle to be drawn both left and right
    cs_L, cs_R = draw_circles(start)
    cg_L, cg_R = draw_circles(goal)

    # dir1 = 'right'
    # dir2 = 'right'
    plt.title(dir1 + " | " + dir2)

    if dir1 == "left":
        cs = cs_L
    else:
        cs = cs_R

    if dir2 == "left":
        cg = cg_L
    else:
        cg = cg_R

    tangent_pts1, tangent_pts2 = find_tangents(cs, cg,dir1,dir2)
    tangent_pts1 = np.array(tangent_pts1).ravel()
    tangent_pts2 = np.array(tangent_pts2).ravel()
    print("target_points: {} | {}".format(tangent_pts1, tangent_pts2))



    #Now to find the best path we will have to do optimization , but for now we can take one of the CSC or CCC paths
    # and find output the final path using RSL trajectory

    #################################
    # FINDING PATH
    #################################
    x_traj = [start[0]]
    y_traj = [start[1]]
    xc1 = cs[0]
    yc1 = cs[1]
    xc2 = cg[0]
    yc2 = cg[1]
    r = 2.33
    """
    THIS IS WHERE THE ACTUAL PATH IS BEING FOUND
    IF WE CAN CHANGE THIS CODE A LITTLE THEN WE CAN GET AWAY FROM TURNING ALL THE WAY
    IT WON'T BE A PERFECT IMPLEMENTATION OF REEDS SHEPP, BUT IT WILL BE BETTER THAN DUBINS
    """
    """
    FIRST CURVED PORTION
    """
    target_angle = m.atan2(tangent_pts1[1]-yc1, tangent_pts1[0]-xc1)
    current_theta = m.atan2(y_traj[-1]-yc1, x_traj[-1]-xc1)
    decision_angle = wrapToPi(target_angle - current_theta) #basically if this is positive than the robot has to turn anticlockwise else clockwise
    if decision_angle > 0:
        sign = 1
    else:
        sign = -1

    while abs(wrapToPi(current_theta - target_angle))>0.1:
        xval = xc1 + r*m.cos(current_theta)
        yval = yc1 + r*m.sin(current_theta)
        current_theta += sign*0.1
        current_theta = wrapToPi(current_theta)
        # plt.plot(xval, yval,'c.')
        # print(xval, yval)
        x_traj.append(xval)
        y_traj.append(yval)


    """
    STRAIGHT PORTION
    """
    x = x_traj[-1]
    y = y_traj[-1]
    x_target, y_target =  tangent_pts2
    x_line1, y_line1 = tangent_pts1

    dist = m.sqrt((x-x_target)**2 + (y - y_target)**2)
    theta_stride = m.atan2((y_target - y_line1),(x_target -  x_line1))

    while dist > 0.05:
        # plt.plot(x, y, 'c.')
        x = x + 0.01*m.cos(theta_stride)
        y = y + 0.01*m.sin(theta_stride)
        x_traj.append(x)
        y_traj.append(y)
        dist = m.sqrt((x-x_target)**2 + (y - y_target)**2)

    """
    LAST CURVED PORTION
    """
    target_angle = m.atan2(goal[1]-yc2, goal[0]-xc2)
    current_theta = m.atan2(y_traj[-1]-yc2, x_traj[-1]-xc2)
    decision_angle = wrapToPi(target_angle - current_theta)
    if decision_angle > 0:
        sign = 1
    else:
        sign = -1
    while abs(wrapToPi(current_theta - target_angle))>0.1:
        xval = xc2 + r*m.cos(current_theta)
        yval = yc2 + r*m.sin(current_theta)
        current_theta += sign*0.1
        current_theta = wrapToPi(current_theta)
        # plt.plot(xval, yval,'c.')
        # print(xval, yval)
        x_traj.append(xval)
        y_traj.append(yval)
    return(x_traj, y_traj)
