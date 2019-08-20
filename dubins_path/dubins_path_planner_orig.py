from __future__ import division
import matplotlib.pyplot as plt
import math as m

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
    plt.gca().add_patch(L1)
    plt.gca().add_patch(R1)
    return(center_L,center_R)

def find_tangents(C1, C2):
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
        plt.gca().add_patch(C3)

        #find the circle centered centered at start and with radius r1 + r2
        x4 = start[0]
        y4 = start[1]
        r4 = 2.33*2
        C4 = plt.Circle((x4, y4), r4, color='g', fill = False)
        plt.gca().add_patch(C4)

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

        plt.plot(x_int1,y_int1,'co')
        plt.plot(x_int2,y_int2,'co')
        """
        first intersection point calc
        """
        #find angle towards to the x_int1 and y_int1
        ang = m.atan2((y_int1-y4),(x_int1-x4))
        x_t1 = x4 + 2.33*m.cos(ang)
        y_t1 = y4 + 2.33*m.sin(ang)
        plt.plot([x4, x_t1],[y4, y_t1])
        tangent_pts1.append([x_t1,y_t1])

        #find the inner point on the next circle C2
        mag = m.sqrt((x_int1-x2)**2 + (y_int1-y2)**2)
        dir = m.atan2((y2-y_int1),(x2-x_int1))
        x_t11 = x_t1 + mag*m.cos(dir)
        y_t11 = y_t1 + mag*m.sin(dir)

        plt.plot([x_t1,x_t11],[y_t1,y_t11])
        tangent_pts2.append([x_t11,y_t11])



        """
        second intersection point calc
        """
        #find angle towards to the x_int2 and y_int2
        ang = m.atan2((y_int2-y4),(x_int2-x4))
        x_t2 = x4 + 2.33*m.cos(ang)
        y_t2 = y4 + 2.33*m.sin(ang)
        plt.plot([x4, x_t2],[y4, y_t2])
        tangent_pts1.append([x_t2, y_t2])

        #find the inner point on the next circle C2
        mag = m.sqrt((x_int2-x2)**2 + (y_int2-y2)**2)
        dir = m.atan2((y2-y_int2),(x2-x_int2))
        x_t22 = x_t2 + mag*m.cos(dir)
        y_t22 = y_t2 + mag*m.sin(dir)

        plt.plot([x_t2,x_t22],[y_t2,y_t22])
        tangent_pts2.append([x_t22, y_t22])
    else:
        print("[INFO] Inner tangents dont exist")

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

    plt.plot([x_out11, x_out21],[y_out11, y_out21])
    plt.plot([x_out12, x_out22],[y_out12, y_out22])
    tangent_pts1.append([x_out11, y_out11])
    tangent_pts1.append([x_out12, y_out12])

    tangent_pts2.append([x_out21, y_out21])
    tangent_pts2.append([x_out22, y_out22])

    return(tangent_pts1, tangent_pts2)




start = [0,0,0]
goal = [5,5,m.pi/2]

plt.axis('scaled')
plt.xlim(-10,10)
plt.ylim(-10,10)
plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='r',head_width=0.5, head_length=1)

#find the center point of the circle to be drawn both left and right
cs_L, cs_R = draw_circles(start)
cg_L, cg_R = draw_circles(goal)

tangent_pts1, tangent_pts2 = find_tangents(cs_R, cg_L,'right','left')


#Now to find the best path we will have to do optimization , but for now we can take one of the CSC or CCC paths
# and find output the final path using RSL trajectory


plt.show()
