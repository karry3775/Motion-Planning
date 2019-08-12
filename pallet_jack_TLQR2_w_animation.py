"""
IMPORTS
"""
from casadi import *
import math as m
import numpy as np
import time
import random
import copy
import matplotlib.pyplot as plt
from draw_diff_drive import draw_robot

ANIMATION = True
"""
FUNCTIONS
"""
def plot_setup():
    plt.xlim(0,10)
    plt.ylim(0,10)

def plot_trajec(U,K):
    X = np.zeros((3,K+1))
    start,goal = robot_pose()
    L = robot_params()
    plt.plot(start[0],start[1],'co',label="start")
    plt.plot(goal[0],goal[1],'*g',label="goal")
    for i in range(K):
        if i==0:
            X[0,i] = start[0]
            X[1,i] = start[1]
            X[2,i] = start[2]
        else:
            X[0,i] = X[0,i-1] + 0.1*U[0,i-1]*m.cos(X[2,i-1] + U[1,i-1])
            X[1,i] = X[1,i-1] + 0.1*U[0,i-1]*m.sin(X[2,i-1] + U[1,i-1])
            X[2,i] = X[2,i-1] + (1/L)*0.1*U[0,i-1]*m.sin(U[1,i-1])
    X[0,K] = X[0,K-1] + 0.1*U[0,K-1]*m.cos(X[2,K-1] + U[1,K-1])
    X[1,K] = X[1,K-1] + 0.1*U[0,K-1]*m.sin(X[2,K-1] + U[1,K-1])
    X[2,K] = X[2,K-1] + (1/L)*0.1*U[0,K-1]*m.sin(U[1,K-1])

    plt.plot(X[0,:],X[1,:],label="nominal_trajectory")
    plt.xlim(0,10)
    plt.ylim(0,10)
    # plt.show()

    print("printing state")
    for i in range(K):
        print(X[0,i],X[1,i],X[2,i])

    return X


def robot_params():
    L = 1
    return L
def robot_pose():
    start = DM([[3],[1],[0]])
    goal = DM([[1],[8],[0]])
    return(start,goal)

def update(X,U,dt):
    L = robot_params()
    X_new = X + dt*blockcat([[mtimes(U[0],cos(X[2]+U[1]))],
                             [mtimes(U[0],sin(X[2]+U[1]))],
                             [mtimes(U[0],(1/L)*sin(U[1]))]])
    return X_new

def kinematic_uncertain(X,U,dt):
    """
    remember these arrays are np.arrays
    """
    X_new = np.zeros((3,1))
    X_new[0] = X[0] + dt*(np.dot(U[0],m.cos(X[2]+U[1])) + random.uniform(-0.1,0.1))
    X_new[1] = X[1] + dt*(np.dot(U[0],m.sin(X[2]+U[1])) + random.uniform(-0.1,0.1))
    X_new[2] = X[2] + dt*(np.dot(U[0],m.sin(U[1])) + random.uniform(-0.1,0.1))

    return X_new

def kinematic(X,U,dt):
    X_new = np.zeros((3,1))
    X_new[0] = X[0] + dt*(U[0]*m.cos(X[2]+U[1]))
    X_new[1] = X[1] + dt*(U[0]*m.sin(X[2]+U[1]))
    X_new[2] = X[2] + dt*(U[0]*m.tan(U[1]))

    return X_new


def cost_function(U,K):
    cost = 0
    dt = 0.1
    start,goal = robot_pose()
    U = reshape(U,2,K)

    R = DM([[20,0],[0,20]])
    Q = DM([[20,0,0],[0,20,0],[0,0,0]])
    Qf = 4000*DM([[5,0,0],[0,5,0],[0,0,5]])

    X = MX(3,K+1)

    for i in range(K):
        if i==0:
            X[:,i] = start[:]
        else:
            X[:,i] = update(X[:,i-1],U[:,i-1],dt)
        cost = (cost + mtimes(mtimes((goal - X[:,i]).T,Q),(goal - X[:,i]))
                    + mtimes(mtimes(U[:,i].T,R),U[:,i]))

    X[:,K] = update(X[:,K-1],U[:,K-1],dt)
    t_cost = mtimes(mtimes((goal - X[:,K]).T,Qf),(goal - X[:,K]))
    cost = cost + t_cost
    return cost

def acceleration_limit(U,K):
    U = reshape(U,2,K)
    c = MX(2,K-1)
    va_max = 0.05
    sa_max = 0.05
    for i in range(K-1):
        c[0,i] = va_max**2 - mtimes((U[0,i+1]-U[0,i]).T,(U[0,i+1]-U[0,i]))
        c[1,i] = sa_max**2 - mtimes((U[1,i+1]-U[1,i]).T,(U[1,i+1]-U[1,i]))

    c = reshape(c,2*(K-1),1)
    return c

def calc_dummy_L(Ak,Bk,Pk1,Wu):
    L_dummy = mtimes(mtimes(mtimes(inv(Wu + mtimes(mtimes(Bk.T,Pk1),Bk)),Bk.T),Pk1),Ak)
    return L_dummy

def calc_P(Ak,Bk,L,Pk1,Wx):
    Pk = mtimes(mtimes(Ak.T,Pk1),Ak) - mtimes(mtimes(mtimes(Ak.T,Pk1),Bk),L) + Wx
    return Pk

if __name__ == "__main__":

    K = 70 #planning horizon
    dt = 0.1 #update interval
    init = DM.ones(2*K,1)
    start,goal = robot_pose()
    L = robot_params()
    #optimization starts
    opti = Opti()
    U = opti.variable(2*K,1)
    # if (start[0] == goal[0]) or (start[1]==goal[1]):
        # print("in use")
        # opti.set_initial(U,init)
    opti.minimize(cost_function(U,K))
    opti.subject_to(acceleration_limit(U,K)>0)
    opti.solver('ipopt')

    sol = opti.solve()
    U = sol.value(U)
    U = reshape(U,2,K)
    U_nom = np.array(copy.deepcopy(U))
    X = plot_trajec(U,K)
    X = DM(X)
    X_nom = np.array(copy.deepcopy(X))

    #first order of business define cost matrices
    Wx = DM([[20,0,0],[0,20,0],[0,0,20]])
    Wu = DM([[20,0],[0,200]])
    Lk_array = np.zeros((2,3,K))

    #second order of business find At, Bt for every time horizon
    i = K
    while i>=0:
        if i==K:
            Pk1 = Wx
            i-=1
            continue
        Ak =  blockcat([[0,0,-mtimes(U[0,i],sin(X[2,i] + U[1,i]))],
                        [0,0, mtimes(U[0,i],cos(X[2,i] + U[1,i]))],
                        [0,0,0]])
        Bk = blockcat([[cos(X[2,i]+U[1,i]),-mtimes(U[0,i],sin(X[2,i] + U[1,i]))],
                       [sin(X[2,i]+U[1,i]), mtimes(U[0,i],cos(X[2,i] + U[1,i]))],
                       [sin(U[1,i])*(1/L), (1/L)*mtimes(U[0,i],cos(U[1,i]))]])

        L_dummy = calc_dummy_L(Ak,Bk,Pk1,Wu)
        Pk = calc_P(Ak,Bk,L_dummy,Pk1,Wx)
        Lk = mtimes(mtimes(mtimes(inv(Wu + mtimes(mtimes(Bk.T,Pk1),Bk)),Bk.T),Pk1),Ak)
        Lk_np = np.array(Lk)
        # print("storing_Lk_array")
        Lk_array[:,:,i] = Lk_np
        i-=1
        Pk1 = Pk
        # print(Ak,Bk)

    #reverse_Lk_array here
    Lk_array_rev = np.zeros((2,3,K))
    for i in range(K):
        Lk_array_rev[:,:,i] = Lk_array[:,:,-1-i]


    """
    sanity check
    """
    print "Lk_array first" , Lk_array[:,:,0]
    print "Lk_array_rev last" , Lk_array_rev[:,:,-1]
    #calculating T-LQR path
    s = np.array(start)
    X_tlqr = np.zeros((3,K+1))
    X_wtlqr = np.zeros((3,K+1))
    u_c = np.zeros((3,1))
    for i in range(K):
        if i==0:
            X_tlqr[0,i] = s[0]
            X_tlqr[1,i] = s[1]
            X_tlqr[2,i] = s[2]
            X_wtlqr[0,i] = s[0]
            X_wtlqr[1,i] = s[1]
            X_wtlqr[2,i] = s[2]
            continue
        if i==1:
            result = kinematic_uncertain(X_tlqr[:,i-1],U_nom[:,i-1],dt)
            X_tlqr[0,i] = result[0]
            X_tlqr[1,i] = result[1]
            X_tlqr[2,i] = result[2]
            X_wtlqr[0,i] = result[0]
            X_wtlqr[1,i] = result[1]
            X_wtlqr[2,i] = result[2]
            # X_a = X_nom[:,i]
            state_error = np.reshape(X_nom[:,i],(3,1)) - np.reshape(X_tlqr[:,i],(3,1))
            u_c = -np.dot(Lk_array_rev[:,:,i-1],state_error)
            # print("uc shape whenn i=1")
            # print(u_c.shape)
            continue

        else:
            new_u = np.reshape(U_nom[:,i-1],(2,1)) + u_c
            # print("unom then uc")
            # print(U_nom[:,i-1].shape)
            # print(u_c.shape)
            result = kinematic_uncertain(X_tlqr[:,i-1],new_u,dt)
            r2 = kinematic_uncertain(X_wtlqr[:,i-1],U_nom[:,i-1],dt)
            X_tlqr[0,i] = result[0]
            X_tlqr[1,i] = result[1]
            X_tlqr[2,i] = result[2]
            X_wtlqr[0,i] = r2[0]
            X_wtlqr[1,i] = r2[1]
            X_wtlqr[2,i] = r2[2]
            # X_a = X_nom[:,i]
            state_error = np.reshape(X_nom[:,i],(3,1)) - np.reshape(X_tlqr[:,i],(3,1))
            u_c = -np.dot(Lk_array_rev[:,:,i-1],state_error)

    new_u = np.reshape(U_nom[:,K-1],(2,1)) + u_c
    result = kinematic_uncertain(X_tlqr[:,K-1],new_u,dt)
    X_tlqr[0,K] = result[0]
    X_tlqr[1,K] = result[1]
    X_tlqr[2,K] = result[2]
    r2 = kinematic_uncertain(X_wtlqr[:,K-1],U_nom[:,K-1],dt)
    X_wtlqr[0,K] = r2[0]
    X_wtlqr[1,K] = r2[1]
    X_wtlqr[2,K] = r2[2]

    if not ANIMATION:
        plt.plot(X_tlqr[0,:],X_tlqr[1,:],'--',label='tlqr_trajectory')
        # plt.plot(X_wtlqr[0,:],X_wtlqr[1,:],'--',label='without_tlqr_trajectory')
        plt.legend()
        plt.show()

    for i in range(1):
        if ANIMATION:
            #show animation
            for i in range(K):
                plt.cla()
                plot_setup()
                plt.plot(start[0],start[1],'co',label="start")
                plt.plot(goal[0],goal[1],'*g',label="goal")
                plt.plot(X_tlqr[0,0:i+1],X_tlqr[1,0:i+1],'--',label='tlqr')
                plt.plot(X_nom[0,0:i+1],X_nom[1,0:i+1],'--',label='Nominal')
                draw_robot(X_tlqr[0,i],X_tlqr[1,i],X_tlqr[2,i])
                plt.pause(0.05)

    plt.legend()
    plt.show()
