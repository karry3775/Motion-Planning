"""
WE ARE USING REAR KINEMATICS
"""
from __future__ import division
from Atsushi_reed_shepp import reeds_shepp_path_planning as RSP
import matplotlib.pyplot as plt
from RSP_TRACKING_FUNC_REAR import path_track4
# from RSP_TRACKING_REAR_W_REAR_PERP import path_track3
import numpy as np
import math as m

def wrapToPi(theta):
    return m.atan2(m.sin(theta),m.cos(theta))

def segregate_paths(x_traj, y_traj):
    visualize = False
    master_path = []
    path = []
    for i in range(len(x_traj)-2):
        first = [x_traj[i], y_traj[i]]
        mid = [x_traj[i+1], y_traj[i+1]]
        last = [x_traj[i+2], y_traj[i+2]]
        check_angle = abs(wrapToPi(m.atan2((last[1]-mid[1]),(last[0]-mid[0])) - m.atan2((mid[1]-first[1]),(mid[0]-first[0]))))
        if visualize:
            plt.plot([first[0],mid[0],last[0]],[first[1],mid[1],last[1]])
            plt.pause(0.1)
        if check_angle > m.pi/2:
            path.append(first)
            path.append(mid)
            master_path.append(path)
            path = []
        else:
            path.append(first)
    master_path.append(path)
    return master_path


def RSP_path(start, goal):
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

def process_path(x_traj, y_traj):
    final_path = []
    for i in range(len(x_traj)):
        final_path.append([x_traj[i],y_traj[i]])
    #APPEND THE PATH SO THAT THE LAST THING BECOMES TRACKABLE
    last_theta = m.atan2((y_traj[-1]-y_traj[-2]),(x_traj[-1]-x_traj[-2]))
    x_ap = x_traj1[-1] + 2*m.cos(last_theta)
    y_ap = y_traj1[-1] + 2*m.sin(last_theta)
    # final_path.append([x_ap, y_ap])
    thetas = m.atan2((y_traj[1]-y_traj[0]),(x_traj[1]-x_traj[0]))

    return(final_path, thetas)


start = [0,0,-m.pi/2]
# goals = [[2.0,-2.0,-m.pi/2],[2.0,-4.0,-m.pi/2],[0.0,-2.0,-m.pi/2],[0.0,-4.0,-m.pi/2]]
# goals = [[2.0,-4.0,-m.pi/2],[0.0,-4.0,-m.pi/2]]
# goals = [[2.0,-3.5,-m.pi/2],[2.0,-4.0,-m.pi/2],[0.0,-3.5,-m.pi/2],[0.0,-4.0,-m.pi/2]]
# goals = [[2.0,-4.0,-m.pi/2],[2.0,-6.0,-m.pi/2],[0.0,-4.0,-m.pi/2],[0.0,-6.0,-m.pi/2
goals = [[2.0,-4.0,-m.pi/2],[2.0,-6.0,-m.pi/2],[0.0,-4.0,-m.pi/2],[5.0,-6.0,-m.pi/2],[5.0,0.0,-m.pi/2]]
theta = start[2]
x_lim = (-5,6)
y_lim = (-7.5,4)
x_traj = []
y_traj = []
"""
SEQUENTIAL PATH PLANNING
"""
for goal in goals:
    px, py = RSP_path(start, goal)
    master_path = segregate_paths(px, py)
    x_traj.extend(px)
    y_traj.extend(py)

    for paths in master_path:
        x,y,theta = path_track4(paths, theta,x_lim, y_lim, x_traj, y_traj)
        path_array = np.array(paths)
        plt.plot(path_array[:,0],path_array[:,1])
    start = [x,y,theta]



"""
ENDS HERE
"""
# x_traj = []
# y_traj = []
# x_traj1 = []
# y_traj1 = []
# x_traj2 = []
# y_traj2 = []
# x_traj3 = []
# y_traj3 = []
# i = 0
# for goal in goals:
#     px, py = RSP_path(start, goal)
#     if i==0:
#         x_traj1 = px
#         y_traj1 = py
#     elif i==1:
#         x_traj2 = px
#         y_traj2 = py
#     elif i==2:
#         x_traj3 = px
#         y_traj3 = py
#     elif i==3:
#         x_traj4 = px
#         y_traj4 = py
#
#     x_traj.extend(px)
#     y_traj.extend(py)
#     start = goal
#     plt.pause(1)
#     i+=1
# plt.cla()
# plt.plot(x_traj, y_traj, 'k--')
# plt.pause(1)
# # plt.show()
#
# """
# THE CODE ABOVE THIS HAS OBTAINED A REEDS SHEPP PATH USING ATSUSHIS PACKAGE
# NOW THE TASK IS TO FIND THE CUSPS AND SEGREGATE INTO THREE DIFFERENT PATHS
# IDEALLY SINCE WE ARE OBTAINING THREE DIFFERENT PATHS WE CAN DIRECTLY USE THEM
# BUT WE WOULD LIKE TO DEVELOP THIS METHODOLOGY TO ADDRESS ANY SITUATION FOR FUTURE
# """
# #As it turns out we need a function than can identify cusps
# master_path = segregate_paths(x_traj, y_traj)
# num_path = 0
# print(len(master_path))
# plt.cla()
# for paths in master_path:
#     path_array = np.array(paths)
#     print(path_array)
#     print(path_array.shape)
#     print("------------------------\n")
#     plt.plot(path_array[:,0],path_array[:,1])
#     # plt.pause(1)
#     num_path+=1
# print("{} paths found!".format(num_path))
# # plt.show()
#
# """
# After obtaining all the subpaths we can do path_track3
# """
# theta = start[2] #was negative before
# x_lim = (-5,6)
# y_lim = (-7.5,4)
# for paths in master_path:
#     theta = path_track3(paths, theta,x_lim, y_lim)
#     path_array = np.array(paths)
#     plt.plot(path_array[:,0],path_array[:,1])
#
# for paths in master_path:
#     path_array = np.array(paths)
#     plt.plot(path_array[:,0],path_array[:,1])

plt.show()

# final_path, thetas = process_path(x_traj1, y_traj1)
# theta = path_track3(final_path, start[2])
# final_path, thetas = process_path(x_traj2, y_traj2)
# theta = path_track3(final_path, theta)
# final_path, thetas = process_path(x_traj3, y_traj3)
# theta = path_track3(final_path, theta)
# final_path, thetas = process_path(x_traj4, y_traj4)
# theta = path_track3(final_path, theta)
#
# plt.show()
