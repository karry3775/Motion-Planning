import matplotlib.pyplot as plt
from dubins_path_planner_function import dubins_path
import random
import math as m

for i in range(5):
    plt.cla()
    plt.axis('scaled')
    plt.xlim(-5,20)
    plt.ylim(-5,20)

    start = [random.uniform(1,2),random.uniform(1,2),random.uniform(-m.pi/2, m.pi/2)]
    goal = [random.uniform(8,9),random.uniform(8,9),random.uniform(-m.pi/2, m.pi/2)]
    print("start: {}| goal: {}".format(start, goal))
    dir1 = 'right'
    dir2 = 'right'

    plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
    plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='r',head_width=0.5, head_length=1)
    x_traj, y_traj = dubins_path(start, goal,dir1,dir2)
    plt.plot(x_traj, y_traj,'k--')
    plt.pause(3)

plt.show()
