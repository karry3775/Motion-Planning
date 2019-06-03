"""
IMPORTS
"""
import casadi
import math as m
import matplotlib.pyplot as plt
import numpy as np
"""
FUNCTIONS
"""
def env_setup():
    plt.xlim(0,10)
    plt.ylim(0,10)
    obs = np.array([[3,4,0.5],[7,8,0.5],[1,8,0.3]])

    for i in range(obs.shape[0]):
        rad = obs[i,2]; x = obs[i,0]; y = obs[i,1]
        c = plt.Circle((x,y),radius = rad,fc='b')
        plt.gca().add_patch(c)

    return obs


def robot_pose_plot():
    start = np.array([2,2,0])
    goal =  np.array([2,6,m.pi/2])
    return (start,goal)

def plot_trajec(U_tot,T):
    start,goal = robot_pose_plot()
    x_start = start[0]
    y_start = start[1]
    x_goal = goal[0]
    y_goal = goal[1]
    env_setup()
    plt.plot(x_start,y_start,'^g')
    plt.plot(x_goal,y_goal,'^k')

    X_tot = np.zeros((3,T))
    for i in range(T):
        if i==0:
            X_tot[:,i] = start[:]
        else:
            x = X_tot[0,i-1]; y=X_tot[1,i-1];theta = X_tot[2,i-1]
            x_new = x + 0.1*U_tot[0,i-1]*m.cos(theta)
            y_new = y + 0.1*U_tot[0,i-1]*m.sin(theta)
            theta_new = theta + 0.1*U_tot[0,i-1]*m.tan(U_tot[1,i-1])
            X_tot[0,i] = x_new; X_tot[1,i] = y_new ; X_tot[2,i]=theta_new

    plt.plot(X_tot[0,:],X_tot[1,:])
    plt.show()

def robot_pose():
    """
    Through this we can access start and end from anywhere
    """
    start = casadi.DM([[2],[2],[0]])
    goal  = casadi.DM([[2],[6],[m.pi/2]])
    return(start,goal)

def robot_params():
    L = 1
    X_max = casadi.DM([[10],[10]]) #works both with DM and not DM
    return(L,X_max)

def update(X,U):
    """
    X contains  [x,y,theta] in a column
    U contains  [v,s] in a column
    """
    x = X[0]; y = X[1] ; theta = X[2]
    v = U[0]; s = U[1]

    dt = 0.1 #update time
    L,_ = robot_params()
    #kinematics
    x_new = x + v*casadi.cos(theta)*dt
    y_new = y + v*casadi.sin(theta)*dt
    theta_new = theta + ((v*casadi.tan(s))/L)*dt

    #lets make it into a casadi thing
    X_new = casadi.blockcat([[x_new],[y_new],[theta_new]])
    return X_new

def find_X_tot(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    X_tot = casadi.MX(3,T)
    start,goal = robot_pose()
    for i in range(T):
        if i == 0:
            X_tot[:,i] = start[:]
        else:
            X_tot[:,i] = update(X_tot[:,i-1],U_tot[:,i-1])

    return X_tot

def obstacle_cost(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    X_tot  = find_X_tot(U_var,T)

    obs = env_setup()
    obs = casadi.DM(obs)
    cost = 0
    for i in range(T):
        x = X_tot[0,i]; y = X_tot[1,i]
        x0 = obs[0,0];y0 = obs[0,1];r0 = obs[0,2]
        x1 = obs[1,0];y1 = obs[1,1];r1 = obs[1,2]
        x2 = obs[2,0];y2 = obs[2,1];r2 = obs[2,2]
        cost = cost + r0*5000*casadi.exp(-((x-x0)**2+(y-y0)**2 - r0**2))
        cost = cost + r1*5000*casadi.exp(-((x-x1)**2+(y-y1)**2 - r1**2))
        cost = cost + r2*5000*casadi.exp(-((x-x2)**2+(y-y2)**2 - r2**2))

    return cost

def total_cost(U_var,T):
    Q = casadi.DM([[20,0,0],[0,20,0],[0,0,3]])
    R = casadi.DM([[20,0],[0,200]])
    Qf = 40000*casadi.DM([[5,0,0],[0,5,0],[0,0,10]])
    cost = 0
    #find X_tot
    U_tot = casadi.reshape(U_var,2,T)
    X_tot = find_X_tot(U_var,T)
    start,goal = robot_pose()

    for i in range(T):
        x = goal - X_tot[:,i];  u = U_tot[:,i]
        cost = cost + casadi.mtimes(casadi.mtimes(x.T,Q),x) + casadi.mtimes(casadi.mtimes(u.T,R),u)

    terminal_cost = casadi.mtimes(casadi.mtimes((goal - X_tot[:,i]).T,Qf),goal - X_tot[:,i])
    obs_cost = obstacle_cost(U_var,T)
    cost =  cost + terminal_cost + obs_cost


    return cost

def boundary_constraints(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    X_tot = find_X_tot(U_var,T)
    c = casadi.MX(2,T)

    _,X_max = robot_params()

    for i in range(T):
        c[:,i] = X_max - X_tot[0:2,i]

    c = casadi.reshape(c,2*T,1)
    return c

def X_min_constaints(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    X_tot = find_X_tot(U_var,T)
    X_var = casadi.reshape(X_tot,3*T,1)
    return X_var

def steering_constraint(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    st_max = m.pi/2
    c = casadi.MX(1,T)

    for i in range(T):
        c[:,i] = st_max - U_tot[1,i]

    return c

def steering_constraint2(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    st_max = m.pi/2
    c = casadi.MX(1,T)

    for i in range(T):
        c[:,i] = st_max + U_tot[1,i]

    return c

def velocity_constraint(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    v_max = 10
    c = casadi.MX(1,T)

    for i in range(T):
        c[:,i] = v_max - U_tot[0,i]

    return c

def velocity_constraint2(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    v_max = 10
    c = casadi.MX(1,T)

    for i in range(T):
        c[:,i] =  U_tot[0,i] + v_max
        # c[:,i] = U_tot[0,i]

    return c

def acceleration_limit(U_var,T):
    U_tot = casadi.reshape(U_var,2,T)
    a_max = 0.05
    v_max = 0.05
    c = casadi.MX(2,T-1)
    for i in range(T-1):
        c[0,i] = a_max**2 - casadi.mtimes((U_tot[0,i+1]-U_tot[0,i]).T,U_tot[0,i+1]-U_tot[0,i])
        c[1,i] = v_max**2 - casadi.mtimes((U_tot[1,i+1]-U_tot[1,i]).T,U_tot[1,i+1]-U_tot[1,i])

    # c[:,T] = casadi.DM([[0.1],[0.1]])
    c = casadi.reshape(c,2*(T-1),1)
    return c

def nominal_trajectory():
    opti = casadi.Opti()
    T = 100
    U_var = opti.variable(2*T,1)
    X_tot = find_X_tot(U_var,T)

    #objective
    opti.minimize(total_cost(U_var,T))
    #subject to
    opti.subject_to(boundary_constraints(U_var,T)>0)
    opti.subject_to(X_min_constaints(U_var,T)>0)
    opti.subject_to(steering_constraint(U_var,T)>0)
    opti.subject_to(steering_constraint2(U_var,T)>0)
    opti.subject_to(velocity_constraint2(U_var,T)>0)
    opti.subject_to(velocity_constraint(U_var,T)>0)
    opti.subject_to(acceleration_limit(U_var,T)>0)

    opti.solver('ipopt')

    sol = opti.solve()
    U = sol.value(U_var)
    U_tot = casadi.reshape(U,2,T)
    print(U_tot)
    plot_trajec(U_tot,T)

nominal_trajectory()
# env_setup()
# plt.show()
