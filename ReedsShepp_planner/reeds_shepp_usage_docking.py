import matplotlib.pyplot as plt
from docking_reeds_shepp_specific import RSP as rsp_flipped
from reeds_shepp_planner_function import RSP as rsp_normal
import random
import math as m

plt.cla()
plt.axis('scaled')
plt.xlim(-5,20)
plt.ylim(-5,20)

# start = [random.uniform(1,2),random.uniform(1,2),random.uniform(-m.pi/2, m.pi/2)]
# goal = [random.uniform(8,9),random.uniform(8,9),random.uniform(-m.pi/2, m.pi/2)]

start = [0,10,m.radians(90)]
goal_orig = [5,0,m.pi/2]

goal = [goal_orig[0], goal_orig[1] + 2, goal_orig[2]]
print("start: {}| goal: {}".format(start, goal_orig))
dir1 = 'right'
dir2 = 'left'

plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='r',head_width=0.5, head_length=1)
plt.arrow(goal_orig[0],goal_orig[1],2*m.cos(goal_orig[2]),2*m.sin(goal_orig[2]),color='r',head_width=0.5, head_length=1)
#check if the start is to the right or left
if start[0] - goal[0]>0:
    """
    which means right
    """
    x_traj, y_traj = rsp_flipped(start, goal,dir1,dir2)
else:
    x_traj, y_traj = rsp_flipped(start, goal,dir1,dir2)

#add more to the trajectory to go to goal
while abs(y_traj[-1] - goal_orig[1])>0.1:
    x_traj.append(x_traj[-1])
    y_traj.append(y_traj[-1] - 0.05)
plt.plot(x_traj, y_traj,'k--')
plt.show()
