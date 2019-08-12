import math as m
import numpy as np
import matplotlib.pyplot as plt
import random

def wrapToPi(theta):
    theta = m.atan2(m.sin(theta),m.cos(theta))
    # theta = ( theta + np.pi) % (2 * np.pi ) - np.pi
    return theta

def draw_vehicle(x,y,theta):
    #base drawing
    a = np.array([0.5,0,1]).T
    b = np.array([-0.5,0.25,1]).T
    c = np.array([-0.5,-0.25,1]).T

    #transform
    T = transform(x,y,theta)
    a = np.matmul(T,a)
    b = np.matmul(T,b)
    c = np.matmul(T,c)

    plt.plot([a[0],b[0]],[a[1],b[1]],'k-')
    plt.plot([a[0],c[0]],[a[1],c[1]],'k-')
    plt.plot([b[0],c[0]],[b[1],c[1]],'k-')
    plt.plot([a[0],x],[a[1],y],'g-')
    plt.plot(x,y,'mo')

def draw_pallet(x,y,theta):
    #base drawing
    a = np.array([0.25,0,1]).T
    b = np.array([0.15,-0.2,1]).T
    c = np.array([0.15,0.2,1]).T
    d = np.array([-0.1,0.2,1]).T
    e = np.array([-0.1,0.1,1]).T
    f = np.array([-0.1,-0.1,1]).T
    g = np.array([-0.1,-0.2,1]).T
    i = np.array([-0.6,0.2,1]).T
    j = np.array([-0.6,0.1,1]).T
    k = np.array([-0.6,-0.1,1]).T
    l = np.array([-0.6,-0.2,1]).T

    T = transform(x,y,theta)
    a = np.matmul(T,a)
    b = np.matmul(T,b)
    c = np.matmul(T,c)
    d = np.matmul(T,d)
    e = np.matmul(T,e)
    f = np.matmul(T,f)
    g = np.matmul(T,g)
    i = np.matmul(T,i)
    j = np.matmul(T,j)
    k = np.matmul(T,k)
    l = np.matmul(T,l)

    plt.plot([a[0],c[0]],[a[1],c[1]],'g')
    plt.plot([a[0],b[0]],[a[1],b[1]],'g')
    plt.plot([d[0],c[0]],[d[1],c[1]],'g')
    plt.plot([g[0],b[0]],[g[1],b[1]],'g')
    plt.plot([d[0],g[0]],[d[1],g[1]],'g')
    plt.plot([d[0],i[0]],[d[1],i[1]],'g')
    plt.plot([g[0],l[0]],[g[1],l[1]],'g')
    plt.plot([k[0],f[0]],[k[1],f[1]],'g')
    plt.plot([j[0],e[0]],[j[1],e[1]],'g')
    plt.plot([j[0],i[0]],[j[1],i[1]],'g')
    plt.plot([k[0],l[0]],[k[1],l[1]],'g')
    plt.plot([x,a[0]],[y,a[1]],'k--')
    plt.plot(x,y,'mo')



def transform(x,y,theta):
    T = np.array([[m.cos(theta),-m.sin(theta),x],
                  [m.sin(theta),m.cos(theta),y],
                  [0,0,1]])
    return T

def plot_theta(pose,col,label):
    x = pose[0]
    y = pose[1]
    theta = pose[2]
    plt.arrow(x,y,m.cos(theta),m.sin(theta),width=0.1,color=col,label=label)

def update(x,y,theta,v,w):
    dt = 0.01
    theta = theta + w*dt
    x = x + v*m.cos(theta)*dt
    y = y + v*m.sin(theta)*dt

    return (x,y,theta)

def PID(start,goal):
    #PID Gains
    Kpv = 9
    Kpa = 15
    Kpb = -3

    x = start[0]
    y = start[1]
    theta = start[2]

    x_g = goal[0]
    y_g = goal[1]
    theta_g = goal[2]

    x_traj = []
    y_traj = []
    theta_traj = []
    rho = m.sqrt((x_g-x)**2 + (y_g-y)**2)
    counter =1
    while rho>0.0001:
        # print(x,y,theta)

        x_traj.append(x)
        y_traj.append(y)
        theta_traj.append(theta)

        line_theta = m.atan2(y_g-y,x_g-x)

        rho = m.sqrt((x-x_g)**2 + (y-y_g)**2)
        alpha = wrapToPi(line_theta-theta)
        beta = wrapToPi(theta_g - theta - alpha)
        # print('errors',rho,alpha,beta)

        v = Kpv*rho
        w = Kpa*alpha + Kpb*beta

        if alpha>m.pi/2 or alpha<-m.pi/2:
            v = -v

        x,y,theta = update(x,y,theta,v,w)
        counter+=1
        if counter>=1000:
            print("[INFO] Goal Not Reached")
            break
        if rho<0.0001:
            print("[INFO] Goal Reached")
            break


    for i in range(len(x_traj)):
        plt.cla()
        plt.xlim(x_traj[i]-2,x_traj[i]+2)
        plt.ylim(y_traj[i]-2,y_traj[i]+2)
        plt.axes().set_aspect('equal','datalim')
        plt.plot(x_traj[0:i],y_traj[0:i],'k--')
        plot_theta(start,'g','start')
        plot_theta(goal,'r','goal')
        draw_pallet(x_traj[i],y_traj[i],theta_traj[i])
        plt.legend()
        plt.pause(0.0001)


def main():
    start = [7,4,0]
    goal = [1,4,0]

    for i in range(10):
        start = [random.uniform(0,10),random.uniform(0,10),random.uniform(-m.pi,m.pi)]
        goal = [random.uniform(0,10),random.uniform(0,10),random.uniform(-m.pi,m.pi)]
        PID(start,goal)
    plt.show()

if __name__=="__main__":
    try:
        main()

    except KeyboardInterrupt:
        pass
