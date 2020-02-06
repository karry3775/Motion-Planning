"""
NOT FINISHED
"""
import matplotlib.pyplot as plt
import random
import math as m
from queue import Queue
from Atsushi_reed_shepp import reeds_shepp_path_planning as RSP

#lets start with a simple rrt without any path storage
def get_theta(x1,y1,x2,y2):
    theta = m.atan2(y2-y1,x2-x1)
    return theta

def get_nearest(x_traj, y_traj, xr, yr):
    best_dist = 999 #random initialization
    best_point = [xr,yr]
    for i in range(len(x_traj)):
        dist = m.sqrt((x_traj[i]-xr)**2 + (y_traj[i]-yr)**2)
        if dist< best_dist:
            best_pt = [x_traj[i],y_traj[i]]
            best_dist = dist

    return best_pt

def RSP_path(start, goal):
    """
    REED SHEPPS RSP
    """
    curvature = 1/2.33
    step_size = 0.1

    px, py, pyaw, mode, clen = RSP(
        start[0], start[1], start[2], goal[0], goal[1], goal[2], curvature, step_size)

    plt.plot(px, py, label="final course " + str(mode))

    # plotting
    plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
    plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='g',head_width=0.5, head_length=1)

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.xlim(-10,10)
    plt.ylim(-10,10)

    return(px,py)


for i in range(1):
    plt.clf()
    start = (5,5,0)
    goal = (random.uniform(0,10), random.uniform(0, 10),random.uniform(-m.pi/2, m.pi/2))
    iter = 0
    buffer = 1000
    x_traj = [start[0]]
    y_traj = [start[1]]
    theta_traj = [start[2]]
    stride = 0.09
    sample_goal_rate = 30

    #plotting starting and end positions
    plt.plot(start[0],start[1],'go')
    plt.plot(goal[0], goal[1], 'ro')
    #Queue based- Remember to use tuples
    tree = Queue()
    tree.put(start)
    cameFrom = {}
    cameFrom[start] = None

    n2goal = m.sqrt((goal[1]-start[1])**2 + (goal[0]-start[1])**2) #neareness to goal

    while n2goal>0.1:
        """
        EVERY ONCE IN A WHILE SAMPLE GOAL POINT AS THE RANDOM POINT
        """
        if random.uniform(0,100) > sample_goal_rate:
            x_rand = random.uniform(0,10)
            y_rand = random.uniform(0,10)
            theta_rand = random.uniform(-m.pi/2, m.pi/2)
        else:
            x_rand = goal[0]
            y_rand = goal[1]
            theta_rand = goal[2]

        """
        SINCE WE ARE DOING NON-HOLONOMIC | WE NEED ANOTHER PLANNER THAN THE STRAIGHT line
        PLANNER WE HAVE BEEN USING
        """
        #draw a RSP to the randomly generated point
        nearest = get_nearest(x_traj,y_traj,x_rand,y_rand)
        """
        CODE TO CHANGE
        """
        RSP_path([],[])


        phi = get_theta(nearest[0],nearest[1],x_rand,y_rand)
        x_gen = nearest[0] + m.cos(phi)*stride
        y_gen = nearest[1] + m.sin(phi)*stride
        """
        """
        x_traj.append(x_gen)
        y_traj.append(y_gen)
        plt.plot(x_gen,y_gen,'c.')
        plt.xlim(0,10)
        plt.ylim(0,10)
        plt.pause(0.01)
        iter+=1

        #tree structure
        tree.put(x_gen,y_gen)
        cameFrom[(x_gen, y_gen)] = (nearest[0], nearest[1])
        #loop break logic
        n2goal = m.sqrt((goal[1]-y_traj[-1])**2 + (goal[0]-x_traj[-1])**2)
        if iter>buffer:
            print("Path not found, buffer exceeded")
            break



    #lets extract the path
    #find nearest to goal
    nearest = get_nearest(x_traj, y_traj, goal[0], goal[1])
    current = (nearest[0], nearest[1]) #extracting whatever came last
    pathx = []
    pathy = []
    while current!=start:
        pathx.append(current[0])
        pathy.append(current[1])
        plt.plot(pathx[-1], pathy[-1], 'r.')
        current = cameFrom[current]
        plt.pause(0.01)

    final_path = []
    for i in range(len(pathx)):
        final_path.append([pathx[i],pathy[i]])

plt.show()
