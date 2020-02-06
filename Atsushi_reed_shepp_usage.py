from Atsushi_reed_shepp import reeds_shepp_path_planning as RSP
import matplotlib.pyplot as plt
import numpy as np
import math as m

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


start = [0,0,m.pi/2]
goals = [[2.0,-4.0,m.pi/2],[0.0,-1.0,m.pi/2],[0.0,-4.0,m.pi/2]]
x_traj = []
y_traj = []
for goal in goals:
    px, py = RSP_path(start, goal)
    x_traj.extend(px)
    y_traj.extend(py)
    start = goal
    plt.pause(1)
plt.cla()
plt.plot(x_traj, y_traj, 'k--')
plt.show()
