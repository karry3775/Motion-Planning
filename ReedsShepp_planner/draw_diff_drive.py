import matplotlib.pyplot as plt
import math
import numpy as np

DISABLE_WHEEL= False
DISABLE_ARROW = False
BIG_CIRCLE = True

def draw_robot(x,y,theta):
    X = np.array([0,0,1]).T
    F = np.array([1,0,1]).T

    a = np.array([-1,1,1]).T
    b = np.array([1,1,1]).T
    c = np.array([1,1.5,1]).T
    d = np.array([-1,1.5,1]).T

    e = np.array([-1,-1.5,1]).T
    f = np.array([1,-1.5,1]).T
    g = np.array([1,-1,1]).T
    h = np.array([-1,-1,1]).T

    #get transformation matrix
    T = np.array([[math.cos(theta),-math.sin(theta),x],
                  [math.sin(theta),math.cos(theta),y],
                  [0,0,1]])

    X = np.matmul(T,X)
    F = np.matmul(T,F)
    a = np.matmul(T,a)
    b = np.matmul(T,b)
    c = np.matmul(T,c)
    d = np.matmul(T,d)
    e = np.matmul(T,e)
    f = np.matmul(T,f)
    g = np.matmul(T,g)
    h = np.matmul(T,h)

    if BIG_CIRCLE:
        base = plt.Circle((X[0],X[1]),radius=1,fc='y')
    else:
        base = plt.Circle((X[0],X[1]),radius=0.2,fc='y')
    plt.gca().add_patch(base)
    if not DISABLE_ARROW:
        plt.plot([X[0],F[0]],[X[1],F[1]],'g') #for plotting arrow
        plt.arrow(X[0],X[1],2*math.cos(theta),2*math.sin(theta),color='g')

    if not DISABLE_WHEEL:
        plt.plot([a[0],b[0]],[a[1],b[1]],'k')
        plt.plot([c[0],b[0]],[c[1],b[1]],'k')
        plt.plot([c[0],d[0]],[c[1],d[1]],'k')
        plt.plot([a[0],d[0]],[a[1],d[1]],'k')

        plt.plot([e[0],f[0]],[e[1],f[1]],'k')
        plt.plot([f[0],g[0]],[f[1],g[1]],'k')
        plt.plot([g[0],h[0]],[g[1],h[1]],'k')
        plt.plot([h[0],e[0]],[h[1],e[1]],'k')

    plt.axis('scaled')
    # plt.xlim(-2,15)
    # plt.ylim(-3,10)
    # plt.show()

def update(x,y,theta,v,w):
    dt = 0.1
    x = x + v*math.cos(theta)*dt
    y = y + v*math.sin(theta)*dt
    theta = theta + w*dt

    return(x,y,theta)

if __name__ == "__main__":
    x = 1
    y = 2
    theta = 0
    x_traj = []
    y_traj = []

    v = 2
    w = 0.5
    for i in range(100):
        plt.cla()
        draw_robot(x,y,theta)
        x_traj.append(x)
        y_traj.append(y)
        plt.plot(x_traj,y_traj,'m--')
        x,y,theta = update(x,y,theta,v,w)
        plt.pause(0.01)

    plt.show()
