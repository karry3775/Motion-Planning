import matplotlib.pyplot as plt
from queue import Queue
#first step is to create a map
def plot_rect(pt,color):
    rect = plt.Rectangle((pt[0]-0.5,pt[1]-0.5),1,1,fc=color,ec='white')
    plt.gca().add_patch(rect)

def plot_map(x_max,y_max,obstacles):
    plt.xlim(0.5,x_max-0.5)
    plt.ylim(0.5,y_max-0.5)
    # plt.axes().set_aspect('equal')
    for obs in obstacles:
        # plt.plot(obs[0],obs[1],'co')
        plot_rect(obs,'g')

    # plt.show()

def get_neighbours(current,obstacles):

    creation_list = [[-1,0],[1,0],[0,1],[0,-1], \
                     [1,1],[-1,-1],[1,-1],[-1,1]]
    neighbours = []
    for creations in creation_list:
        x_n = current[0] + creations[0]
        y_n = current[1] + creations[1]
        if [x_n,y_n] in obstacles:
            continue
        if x_n>=20 or x_n<=0 or y_n>=20 or y_n<=0:
            continue
        neighbours.append((x_n,y_n))

    return neighbours

def animate_robot(pt):
    pass

def main():
    plt.figure(figsize=(10,8))
    x_max = 20
    y_max = 20
    obstacles = [[10,0],[10,1],[10,2],[10,3], \
                 [9,3],[8,3],[7,3],[6,3], \
                 [6,4],[6,5],[6,6],[6,7], \
                 [6,15],[7,15],[8,15],[9,15], \
                 [10,15],[11,15],[12,15],[13,15],
                 [13,14],[13,13],[13,12],[13,11],
                 [10,8],[10,9],[10,10],[10,11],
                 [11,8],[12,8],[13,8],[14,8]
                 ,[15,8],[16,8],[16,9],[16,10],
                 [16,11],[16,12],[16,13],[16,14],
                 [16,15],[16,16]]

    plot_map(x_max,y_max,obstacles)
    start = (2,2)
    goal = (14,12)
    plt.plot(start[0],start[1],'ro')
    plt.plot(goal[0],goal[1],'bo')
    frontier = Queue()
    frontier.put(start)
    cameFrom = {}
    cameFrom[start] = None

    while not frontier.empty():
        current = frontier.get()
        plt.plot(current[0],current[1],'co')

        if current == goal:
            print("Goal Reached")
            break

        for next in get_neighbours(current,obstacles):
            if next not in cameFrom:
                plt.plot(next[0],next[1],'go')
                frontier.put(next)
                cameFrom[next] = current
        plt.pause(0.01)

    current = goal
    pathx = []
    pathy = []
    while current!=start:
        pathx.append(current[0])
        pathy.append(current[1])
        plt.plot(pathx[-1],pathy[-1],'ro')
        current = cameFrom[current]
        plt.pause(0.01)

    pathx.append(start[0])
    pathy.append(start[1])
    pathx.reverse()
    pathy.reverse()
    print(pathx)
    print(pathy)
    plt.plot(pathx,pathy,'g')
    plt.pause(0.01)

    plt.show()

if __name__ == "__main__":
    main()
