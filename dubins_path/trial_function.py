import matplotlib.pyplot as plt
import math as m

start = [1.93, 2.70, -0.46]
goal = [6.45, 7.09, -1.1]

plt.xlim(0,10)
plt.ylim(0,10)
plt.arrow(start[0],start[1],2*m.cos(start[2]),2*m.sin(start[2]),color='g',head_width=0.5, head_length=1)
plt.arrow(goal[0],goal[1],2*m.cos(goal[2]),2*m.sin(goal[2]),color='r',head_width=0.5, head_length=1)

plt.show()
