import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch
from map import MapParameters
import math
import numpy as np
# TODO part of draw


def draw(x,y,yaw,car,mapParameters):
    """
    Parameters:
        x: all x coords of found trajectory
        y: all y coords of found trajectory
        yaw: all yaw angles of found trajectory
        car: car parameters
        mapParameters: mapParams used for search including obstacles
    """

    obstacleX = mapParameters.obstacleX
    obstacleY = mapParameters.obstacleY

    fig, ax = plt.subplots()

    ax.set_title("Hybrid A*")
    #ax.cla()
    ax.set_xlim(min(obstacleX), max(obstacleX)) 
    ax.set_ylim(min(obstacleY), max(obstacleY))
    
    ax.plot(obstacleX, obstacleY, "sk")
    ax.plot(x, y, linewidth=1.5, color='r', zorder=0)
    
    line, = ax.plot([], [], color='black')

    global arrow
    arrow = FancyArrowPatch((0,0),(0,0), 
                            color="blue", 
                            arrowstyle='->,head_width=3.0,head_length=5.0',
                            linewidth=2)
    ax.add_patch(arrow)

    def update_car(carParams, x, y, yaw):
        car = np.array([[-carParams.axleToBack, -carParams.axleToBack, carParams.axleToFront, carParams.axleToFront, -carParams.axleToBack],
                        [carParams.width / 2, -carParams.width / 2, -carParams.width / 2, carParams.width / 2, carParams.width / 2]])

        rotationZ = np.array([[math.cos(yaw), -math.sin(yaw)],
                              [math.sin(yaw), math.cos(yaw)]])
        car = np.dot(rotationZ, car)
        car += np.array([[x], [y]])

        line.set_data(car[0, :], car[1, :])

        return line

    def update_arrow(x1,y1,x2,y2):
        global arrow
        
        print(f"x1: {x1} y1: {y1} x2: {x2} y2: {y2}")

        arrow.remove()
        arrow = FancyArrowPatch((x1, y1), (x2, y2), 
                                color="blue", 
                                arrowstyle='->,head_width=3.0,head_length=5.0',
                                linewidth=2)
        ax.add_patch(arrow)

        return arrow
    
    # update function
    def animate(k):
        line = update_car(car, x[k], y[k], yaw[k])
        arrow = update_arrow(x[k], y[k], x[k] + 4 * math.cos(yaw[k]), y[k] + 4 * math.sin(yaw[k]))

        return line,arrow



    ani = animation.FuncAnimation(
        fig, animate, interval=50, blit=True, frames=len(x))

    ani.save("hybridastar.mp4")

    plt.show()



    # Draw Start, Goal Location Map and Path
    # plt.arrow(s[0], s[1], 1*math.cos(s[2]), 1*math.sin(s[2]), width=.1)
    # plt.arrow(g[0], g[1], 1*math.cos(g[2]), 1*math.sin(g[2]), width=.1)
    # plt.xlim(min(obstacleX), max(obstacleX)) 
    # plt.ylim(min(obstacleY), max(obstacleY))
    # plt.plot(obstacleX, obstacleY, "sk")
    # plt.plot(x, y, linewidth=2, color='r', zorder=0)
    # plt.title("Hybrid A*")


    # Draw Path, Map and Car Footprint
    # plt.plot(x, y, linewidth=1.5, color='r', zorder=0)
    # plt.plot(obstacleX, obstacleY, "sk")
    # for k in np.arange(0, len(x), 2):
    #     plt.xlim(min(obstacleX), max(obstacleX)) 
    #     plt.ylim(min(obstacleY), max(obstacleY))
    #     drawCar(x[k], y[k], yaw[k])
    #     plt.arrow(x[k], y[k], 1*math.cos(yaw[k]), 1*math.sin(yaw[k]), width=.1)
    #     plt.title("Hybrid A*")