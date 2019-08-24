"""
PATH TRACKING USING MPC
"""
import matplotlib.pyplot as plt
from drawing_pallet_jack_rev import dpj
import casadi
import math as m

"""
FUNCTIONS
"""
def update_rear(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x+= (v/2)*(m.cos(theta-s) + m.cos(theta+s))*dt
    y+= (v/2)*(m.sin(theta-s) + m.sin(theta+s))*dt
    theta+= -(v/L)*m.sin(s)*dt
    return(x,y,theta)

def update_rear_casadi(x,y,theta,v,s):
    dt = 0.1
    L = 2.33
    x+= (v/2)*(casadi.cos(theta-s) + casadi.cos(theta+s))*dt
    y+= (v/2)*(casadi.sin(theta-s) + casadi.sin(theta+s))*dt
    theta+= -(v/L)*casadi.sin(s)*dt
    return(x,y,theta)

def cost_function(V,S,start,v,s,path_function):
    #unpacking curve parameters
    slope, intercept = path_function
    #note here that start is a list
    # and V,S are casadi variables
    x_traj = []
    y_traj = []
    theta_traj = []

    x = start[0]
    y = start[1]
    theta = start[2]
    x,y,theta = update_rear(x,y,theta,v,s)
    x_traj.append(x)
    y_traj.append(y)
    theta_traj.append(theta)
    for i in range(V.shape[0]):
        x,y,theta = update_rear_casadi(x,y,theta,V[i],S[i])
        x_traj.append(x)
        y_traj.append(y)
        theta_traj.append(theta)

    #Now that we have the future points based on variable
    #we can define the cost function
    """
    CROSS TRACK ERROR
    """
    cross_track_error = 0
    for i in range(len(x_traj)):
        error = (slope*x_traj[i] + intercept) - (y_traj[i])
        cross_track_error+=error

    return(cost)

def iterative_optimization(path_function,start,v,s):
    """
    OPTIMIZATION
    """
    K = 9 #time steps
    opti = casadi.Opti()
    V = opti.variable(K,1)
    S = opti.variable(K,1)

    cost_function(V,S,start,v,s,path_function)
    # opti.minimize(cost_function(V,S,start,v,s))
    # opti.subject_to(acceleration_limit(U,K)>0)
    # opti.solver('ipopt')
    #
    # sol = opti.solve()
    # U = sol.value(U)



"""
GETTING A LINE TO TRACK
"""
#lets cut ourselves some slack and track just a straight line
# start_x = 1
# end_x = 4
#
# slope = 1
# intercept = 5
# start_y = slope*start_x + intercept
# end_y = slope*end_x + intercept
#
# plt.plot(start_x,start_y,'co',label = "START")
# plt.plot(end_x,end_y,'ro',label = "GOAL")
# plt.plot([start_x,end_x],[start_y,end_y],'m--',label="TARGET PATH")
#
# """
# DEFINE WHERE OUR ROBOT STARTS
# """
# start = [1,6,0]
# #intial controls
# v = 0
# s = 0
# #drawing robot
# dpj(start[0],start[1],start[2],s)
#
#
#
#
#
#
#
#
# plt.xlim(0,10)
# plt.ylim(0,10)
# plt.grid()
# plt.legend()
# plt.show()

"""
FUNCTION CALLING FOR DEBUGGING
"""
start = [1,6,0]
slope = 1
intercept = 5
v = 0.325
s = 0
path_function = [slope, intercept]
iterative_optimization(path_function,start,v,s)
