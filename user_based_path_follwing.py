from pid_path_follow_pallet_jack import path_track2
import matplotlib.pyplot as plt
import math as m

class LineBuilder:
    def __init__(self, line):
        plt.xlim(-10,15)
        plt.ylim(-10,15)
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('click to build line segments')
line, = ax.plot([0], [0])  # empty line
linebuilder = LineBuilder(line)
plt.show()

xs = linebuilder.xs
ys = linebuilder.ys

path = []
for i in range(len(xs)):
    path.append([xs[i],ys[i]])
# """
# TRACKING BY SAMPLING
# """
# #need to properly sample paths for effective tracking
# final_path = []
# x,y = path[0]
# final_path.append([x,y])
# for x,y in path:
#     xf,yf = final_path[-1]
#     if m.sqrt((xf-x)**2 + (yf-y)**2)>3:
#         final_path.append([x,y])
#     else:
#         continue

path_track2(path)
plt.show()
