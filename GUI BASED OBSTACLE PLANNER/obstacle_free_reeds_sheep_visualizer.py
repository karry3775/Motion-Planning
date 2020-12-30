import matplotlib.pyplot as plt
import math as m
from matplotlib.widgets import Slider, Button, RadioButtons
from Atsushi_reed_shepp import reeds_shepp_path_planning as RSP

"""
REFERENCES: https://matplotlib.org/gallery/widgets/slider_demo.html
"""
def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def plot_setup(ax):
    ax.cla()
    ax.set_title("Adjustable Reeds-Shepp Implementation")
    ax.set_aspect('equal')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)

# create a updating plot
def update(val):
    angle1 = sAngle1.val
    angle2 = sAngle2.val
    radius = sRadius.val
    pos1x = sPos1x.val
    pos2x = sPos2x.val
    pos1y = sPos1y.val
    pos2y = sPos2y.val
    # for now we will draw an array which changes in angle
    plot_setup(ax)

    ax.arrow(pos1x, pos1y, m.cos(wrapToPi(m.radians(angle1))), m.sin(wrapToPi(m.radians(angle1))), head_width = 0.1, head_length = 0.5)
    ax.arrow(pos2x, pos2y, m.cos(wrapToPi(m.radians(angle2))), m.sin(wrapToPi(m.radians(angle2))), head_width = 0.1, head_length = 0.5)

    # now we have got angles and we know start and end exactly
    start = [pos1x, pos1y, wrapToPi(m.radians(angle1))]
    goal = [pos2x, pos2y, wrapToPi(m.radians(angle2))]

    px, py  = RSP_path(start, goal, radius)

    # now we need to plot it
    ax.plot(px, py, 'r')


def reset(event):
    sAngle1.reset()
    sAngle2.reset()
    sRadius.reset()
    sPos1x.reset()
    sPos2x.reset()
"""
RSP path function imported from Atsushi_reed_shepp
"""
def RSP_path(start, goal, radius = 2.33):
    curvature = 1/radius #increased the curvature changed from 1/10
    step_size = 0.1

    px, py, pyaw, mode, clen = RSP(
        start[0], start[1], start[2], goal[0], goal[1], goal[2], curvature, step_size)
    return(px,py)

if __name__ == "__main__":
    try:
        # Create a plt figure that has slider to choose angle
        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom = 0.45, left = 0.1)
        plot_setup(ax)

        #create a axes
        axcolor = "lightgoldenrodyellow"
        axAngle1 = plt.axes([0.2, 0.35, 0.2, 0.01], facecolor = axcolor) #[distance from left, distance from bottom, length, width]
        axPos1x = plt.axes([0.2, 0.30, 0.2, 0.01], facecolor = axcolor)
        axPos1y = plt.axes([0.2, 0.25, 0.2, 0.01], facecolor = axcolor)

        axAngle2 = plt.axes([0.6, 0.35, 0.2, 0.01], facecolor = axcolor)
        axPos2x = plt.axes([0.6, 0.30, 0.2, 0.01], facecolor = axcolor)
        axPos2y = plt.axes([0.6, 0.25, 0.2, 0.01], facecolor = axcolor)

        axRadius = plt.axes([0.2, 0.20, 0.2, 0.01], facecolor = axcolor)




        # create a slider
        sAngle1 = Slider(axAngle1, 'Yaw 1 (deg)', 0, 360, valinit = 0, valstep = 5)
        sAngle2 = Slider(axAngle2, 'Yaw 2 (deg)', 0, 360, valinit = 90, valstep = 5)
        sRadius = Slider(axRadius, 'Radius (m)',0, 2.33, valinit = 2.33, valstep = 0.1)
        sPos1x = Slider(axPos1x, 'pos1 x', -10, 10, valinit = 0, valstep = 0.1)
        sPos2x = Slider(axPos2x, 'pos2 x', -10, 10, valinit = 5, valstep = 0.1)
        sPos1y = Slider(axPos1y, 'pos1 y', -10, 10, valinit = 0, valstep = 0.1)
        sPos2y = Slider(axPos2y, 'pos2 y', -10, 10, valinit = 5, valstep = 0.1)
        #create a button
        resetax = plt.axes([0.8, 0.05, 0.1, 0.04])
        resetButton = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')

        sAngle1.on_changed(update)
        sAngle2.on_changed(update)
        sRadius.on_changed(update)
        sPos1x.on_changed(update)
        sPos2x.on_changed(update)
        sPos1y.on_changed(update)
        sPos2y.on_changed(update)
        resetButton.on_clicked(reset)
        plt.show()
    except Exception(), e:
        print("Error {} in try block".format(e))
