import matplotlib.pyplot as plt
import numpy as np
import math as m
from matplotlib.widgets import Slider, Button, RadioButtons
from Atsushi_reed_shepp import reeds_shepp_path_planning as RSP

"""
Utility Functions
"""
def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def RSP_path(start, goal, radius = 2.33):
    curvature = 1/radius #increased the curvature changed from 1/10
    step_size = 0.1

    px, py, pyaw, mode, clen = RSP(
        start[0], start[1], start[2], goal[0], goal[1], goal[2], curvature, step_size)
    return(px,py)

# first of all we need to put obstacles in an environment
class Environment():
    def __init__(self, xBoundary, yBoundary, obs):
        self.xb = xBoundary # of the form [ax, bx]
        self.yb = yBoundary # of the form [ay, by]
        self.obstacles = obs # of the form [[x0, y0,r0],[x1, y1, r1],[x2, y2, r2],...]
        print(self.obstacles)
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(bottom = 0.45, left = 0.1)
        #setting the sliders
        self.axcolor = "lightgoldenrodyellow"
        # setting sliders for start pose
        self.axAngle1 = plt.axes([0.2, 0.35, 0.2, 0.01], facecolor = self.axcolor) #[distance from left, distance from bottom, length, width]
        self.axPos1x = plt.axes([0.2, 0.30, 0.2, 0.01], facecolor = self.axcolor)
        self.axPos1y = plt.axes([0.2, 0.25, 0.2, 0.01], facecolor = self.axcolor)
        # setting sliders for goal pose
        self.axAngle2 = plt.axes([0.6, 0.35, 0.2, 0.01], facecolor = self.axcolor)
        self.axPos2x = plt.axes([0.6, 0.30, 0.2, 0.01], facecolor = self.axcolor)
        self.axPos2y = plt.axes([0.6, 0.25, 0.2, 0.01], facecolor = self.axcolor)
        #setting the radius
        self.axRadius = plt.axes([0.2, 0.20, 0.2, 0.01], facecolor = self.axcolor)
        #creating sliders
        self.sAngle1 = Slider(self.axAngle1, 'Yaw 1 (deg)', 0, 360, valinit = 0, valstep = 5)
        self.sAngle2 = Slider(self.axAngle2, 'Yaw 2 (deg)', 0, 360, valinit = 90, valstep = 5)
        self.sRadius = Slider(self.axRadius, 'Radius (m)',0, 2.33, valinit = 2.33, valstep = 0.1)
        self.sPos1x = Slider(self.axPos1x, 'pos1 x', -10, 10, valinit = 0, valstep = 0.1)
        self.sPos2x = Slider(self.axPos2x, 'pos2 x', -10, 10, valinit = 5, valstep = 0.1)
        self.sPos1y = Slider(self.axPos1y, 'pos1 y', -10, 10, valinit = 0, valstep = 0.1)
        self.sPos2y = Slider(self.axPos2y, 'pos2 y', -10, 10, valinit = 5, valstep = 0.1)
        #create a reset button
        self.resetax = plt.axes([0.8, 0.05, 0.1, 0.04])
        self.resetButton = Button(self.resetax, 'Reset', color = self.axcolor, hovercolor='0.975')


    def plot_setup(self):
        self.ax.cla()
        self.ax.set_xlim(self.xb[0], self.xb[1])
        self.ax.set_aspect('equal')
        self.ax.set_ylim(self.yb[0], self.yb[1])
        self.plot_obstacles()

    def update(self, val):
        angle1 = self.sAngle1.val
        angle2 = self.sAngle2.val
        radius = self.sRadius.val
        pos1x = self.sPos1x.val
        pos2x = self.sPos2x.val
        pos1y = self.sPos1y.val
        pos2y = self.sPos2y.val

        self.plot_setup()

        self.ax.arrow(pos1x, pos1y, m.cos(wrapToPi(m.radians(angle1))), m.sin(wrapToPi(m.radians(angle1))), head_width = 0.1, head_length = 0.5)
        self.ax.arrow(pos2x, pos2y, m.cos(wrapToPi(m.radians(angle2))), m.sin(wrapToPi(m.radians(angle2))), head_width = 0.1, head_length = 0.5)

        # now we have got angles and we know start and end exactly
        start = [pos1x, pos1y, wrapToPi(m.radians(angle1))]
        goal = [pos2x, pos2y, wrapToPi(m.radians(angle2))]

        px, py  = RSP_path(start, goal, radius)

        # now we need to plot it
        self.ax.plot(px, py, 'r')

    def reset_it(self, event):
        self.sAngle1.reset()
        self.sAngle2.reset()
        self.sRadius.reset()
        self.sPos1x.reset()
        self.sPos2x.reset()

    def set_sliders(self):
        # set the update loop
        self.sAngle1.on_changed(self.update)
        self.sAngle2.on_changed(self.update)
        self.sRadius.on_changed(self.update)
        self.sPos1x.on_changed(self.update)
        self.sPos2x.on_changed(self.update)
        self.sPos1y.on_changed(self.update)
        self.sPos2y.on_changed(self.update)
        self.resetButton.on_clicked(self.reset_it)

    def plot_obstacles(self):
        # source : https://stackoverflow.com/questions/9215658/plot-a-circle-with-pyplot
        for ob in self.obstacles:
            c = plt.Circle((ob[0], ob[1]), ob[2], color = 'k')
            self.ax.add_artist(c)


if __name__ == "__main__":
    xBoundary = [-10, 10]
    yBoundary = [-10, 10]
    obstacles = [[0,-5,2],[-5, 5,1],[1,7,1],[5, 0, 1]]
    env = Environment(xBoundary, yBoundary, obstacles)
    env.plot_setup()
    env.plot_obstacles()
    env.set_sliders()
    plt.show()
