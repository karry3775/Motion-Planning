import matplotlib.pyplot as plt
from docking_reeds_shepp_specific import RSP as rsp_flipped
from reeds_shepp_planner_function import RSP as rsp_normal
from RSP_TRACKING_FUNC_REAR import path_track3
import random
import math as m

plt.cla()
plt.axis('scaled')
plt.xlim(-5,20)
plt.ylim(-5,20)

start = [0,15,m.radians(90)]
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
"""
PLACEMENT BEGINS
"""
start = [x_traj[-1],y_traj[-1],goal_orig[2]]
goal_orig = [x_traj[-1]-3,y_traj[-1],goal_orig[2]]

goal = [goal_orig[0], goal_orig[1] + 6, goal_orig[2]]
print("start: {}| goal: {}".format(start, goal_orig))
dir1 = 'right'
dir2 = 'left'

plt.arrow(start[0],start[1],0.5*m.cos(start[2]),0.5*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
plt.arrow(goal[0],goal[1],0.5*m.cos(goal[2]),0.5*m.sin(goal[2]),color='r',head_width=0.5, head_length=1)
plt.arrow(goal_orig[0],goal_orig[1],2*m.cos(goal_orig[2]),2*m.sin(goal_orig[2]),color='r',head_width=0.5, head_length=1)
#check if the start is to the right or left
if start[0] - goal[0]>0:
    """
    which means right
    """
    x_traj1, y_traj1 = rsp_normal(start, goal,dir1,dir2)
else:
    x_traj1, y_traj1 = rsp_normal(start, goal,dir1,dir2)

# for i in range(len(x_traj1)):
#     x_traj.append(x_traj1[i])
#     y_traj.append(y_traj1[i])

#add more to the trajectory to go to goal
while abs(y_traj[-1] - goal_orig[1])>0.1:
    x_traj.append(x_traj[-1])
    y_traj.append(y_traj[-1] - 0.05)

plt.plot(x_traj, y_traj,'k--')

final_path = []
for i in range(len(x_traj)):
    final_path.append([x_traj[i],y_traj[i]])
#APPEND THE PATH SO THAT THE LAST THING BECOMES TRACKABLE
last_theta = m.atan2((y_traj[-1]-y_traj[-2]),(x_traj[-1]-x_traj[-2]))
x_ap = x_traj[-1] + 2*m.cos(last_theta)
y_ap = y_traj[-1] + 2*m.sin(last_theta)
final_path.append([x_ap, y_ap])
thetas = m.atan2((y_traj[1]-y_traj[0]),(x_traj[1]-x_traj[0]))
path_track3(final_path,-thetas)

final_path = []
for i in range(len(x_traj1)):
    final_path.append([x_traj1[i],y_traj1[i]])
#APPEND THE PATH SO THAT THE LAST THING BECOMES TRACKABLE
last_theta = m.atan2((y_traj1[-1]-y_traj1[-2]),(x_traj1[-1]-x_traj1[-2]))
x_ap = x_traj1[-1] + 2*m.cos(last_theta)
y_ap = y_traj1[-1] + 2*m.sin(last_theta)
final_path.append([x_ap, y_ap])
thetas = m.atan2((y_traj1[1]-y_traj1[0]),(x_traj1[1]-x_traj1[0]))
path_track3(final_path,-thetas)

plt.show()
