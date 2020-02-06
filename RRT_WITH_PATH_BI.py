import matplotlib.pyplot as plt
import random
import math as m
from queue import Queue
#lets start with a simple rrt without any path storage
def get_theta(x1,y1,x2,y2):
    theta = m.atan2(y2-y1,x2-x1)
    return theta

def get_nearest(x_traj_s, y_traj_s, xr, yr):
    best_dist = 999 #random initialization
    best_point = [xr,yr]
    for i in range(len(x_traj_s)):
        dist = m.sqrt((x_traj_s[i]-xr)**2 + (y_traj_s[i]-yr)**2)
        if dist< best_dist:
            best_pt = [x_traj_s[i],y_traj_s[i]]
            best_dist = dist

    return best_pt

for i in range(5):
    plt.clf()
    start = (5,5)
    goal = (random.uniform(0,10), random.uniform(0, 10))
    iter = 0
    buffer = 1000
    x_traj_s = [start[0]]
    y_traj_s = [start[1]]
    x_traj_g = [goal[0]]
    y_traj_g = [goal[1]]
    stride = 0.09
    sample_goal_rate = 5

    #plotting starting and end positions
    plt.plot(start[0],start[1],'go')
    plt.plot(goal[0], goal[1], 'ro')
    #Queue based- Remember to use tuples
    tree1 = Queue()
    tree2 = Queue()
    tree1.put(start)
    tree2.put(goal)
    cameFrom1 = {}
    cameFrom2 = {}
    cameFrom1[start] = None
    cameFrom2[goal] = None

    n2goal = m.sqrt((goal[1]-start[1])**2 + (goal[0]-start[1])**2) #neareness to goal
    n2start = m.sqrt((goal[1]-start[1])**2 + (goal[0]-start[1])**2) #neareness to start
    n2each_other = m.sqrt((goal[1]-start[1])**2 + (goal[0]-start[1])**2)

    while n2each_other>0.1:
        """
        EVERY ONCE IN A WHILE SAMPLE GOAL POINT AS THE RANDOM POINT
        """
        if random.uniform(0,100) > sample_goal_rate:
            x_rands = random.uniform(0,10)
            y_rands = random.uniform(0,10)
            x_randg = random.uniform(0,10)
            y_randg = random.uniform(0,10)
        else:
            x_rands = goal[0]
            y_rands = goal[1]
            x_randg = start[0]
            y_randg = start[1]

        #draw a line to the randomly generated point
        nearest_s = get_nearest(x_traj_s,y_traj_s,x_rands,y_rands)
        phi_s = get_theta(nearest_s[0],nearest_s[1],x_rands,y_rands)
        x_gen_s = nearest_s[0] + m.cos(phi_s)*stride
        y_gen_s = nearest_s[1] + m.sin(phi_s)*stride

        nearest_g = get_nearest(x_traj_g,y_traj_g,x_gen_s,y_gen_s)
        phi_g = get_theta(nearest_g[0],nearest_g[1],x_gen_s,y_gen_s)

        x_gen_g = nearest_g[0] + m.cos(phi_g)*stride
        y_gen_g = nearest_g[1] + m.sin(phi_g)*stride

        x_traj_s.append(x_gen_s)
        y_traj_s.append(y_gen_s)

        x_traj_g.append(x_gen_g)
        y_traj_g.append(y_gen_g)

        plt.plot(x_gen_s,y_gen_s,'c.')
        plt.plot(x_gen_g,y_gen_g,'g.')
        plt.xlim(0,10)
        plt.ylim(0,10)
        plt.pause(0.01)
        iter+=1

        #tree1 structure
        tree1.put(x_gen_s,y_gen_s)
        cameFrom1[(x_gen_s, y_gen_s)] = (nearest_s[0], nearest_s[1])
        #loop break logic
        n2goal = m.sqrt((goal[1]-y_traj_s[-1])**2 + (goal[0]-x_traj_s[-1])**2)
        n2n2each_other = m.sqrt((y_traj_g[-1]-y_traj_s[-1])**2 + (x_traj_g[-1]-x_traj_s[-1])**2)
        if iter>buffer:
            print("Path not found, buffer exceeded")
            break

    CONNECTION = True
    while True:
        if CONNECTION:
            x_rand = goal[0]
            y_rand = goal[1]





    """
    EXTRACTION OF THE PATH
    """
    #lets extract the path
    #find nearest_s to goal
    nearest_s = get_nearest(x_traj_s, y_traj_s, goal[0], goal[1])
    current = (nearest_s[0], nearest_s[1]) #extracting whatever came last
    pathx = []
    pathy = []
    while current!=start:
        pathx.append(current[0])
        pathy.append(current[1])
        plt.plot(pathx[-1], pathy[-1], 'r.')
        current = cameFrom1[current]
        plt.pause(0.01)

    final_path = []
    for i in range(len(pathx)):
        final_path.append([pathx[i],pathy[i]])

plt.show()
