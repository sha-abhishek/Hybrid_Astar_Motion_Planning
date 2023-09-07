import numpy as np
from car import Car
from utils import *
import matplotlib.pyplot as plt
from hybrid_astar import *    
    

def main():
    grid_dimension = [0,0,70,42]
    cell_size = 0.5
    car_obj = Car()
    start_conf = (2,35,-np.pi/4)
    goal_conf = (51,8,-np.pi/2)

    lane_bottom = [[8,2,8.5,10],[12.5,2,13,10],[17,2,17.5,10],[21.5,2,22,10],[26,2,26.5,10]
           ,[30.5,2,31,10],[35,2,35.5,10],[39.5,2,40,10],[44,2,44.5,10],[48.5,2,49,10],
           [52.5,2,53,10],[57,2,57.5,10]]
    
    lane_top = [[8,32,8.5,40],[12.5,32,13,40],[17,32,17.5,40],[21.5,32,22,40],[26,32,26.5,40]
           ,[30.5,32,31,40],[35,32,35.5,40],[39.5,32,40,40],[44,32,44.5,40],[48.5,32,49,40],
           [52.5,32,53,40],[57,32,57.5,40]]
    
    obs = [[0,20,30,22],[40,20,65,22]]

    box = lane_bottom + lane_top + obs

    output = hybrid_astar(grid_dimension,cell_size,start_conf,goal_conf,car_obj,obs)

    if len(output)<3:
        path_astar, open = output
        path_astar.insert(0,start_conf)
        total_path = path_astar
        path_dub = []
    else:
        path_astar, path_dub, open = hybrid_astar(grid_dimension,cell_size,start_conf,goal_conf,car_obj,obs)
        path_astar.append(path_dub[0])
        path_astar.insert(0,start_conf)
        total_path = []
        total_path = path_astar + path_dub

    # plot boundary
    xmin = 0
    ymin = 0
    xmax = 70
    ymax = 40
    width = xmax - xmin
    height = ymax - ymin
    rect = plt.Rectangle((xmin, ymin), width, height, linewidth=1, edgecolor='k', facecolor='none')
    plt.gca().add_patch(rect)
    plt.grid()

    # plot start and end configuration
    ang1 = start_conf[2]
    x1 = start_conf[0]
    y1 = start_conf[1]
    arrow_end_x1 = 3 * np.cos(ang1)
    arrow_end_y1 = 3 * np.sin(ang1)
    plt.arrow(x1,y1,arrow_end_x1,arrow_end_y1,width =0.5, head_width=1, head_length=1,color='blue')

    ang2 = goal_conf[2]
    x2 = goal_conf[0]
    y2 = goal_conf[1]
    arrow_end_x2 = 3 * np.cos(ang2)
    arrow_end_y2 = 3 * np.sin(ang2)
    plt.arrow(x2,y2,arrow_end_x2,arrow_end_y2,width =0.5, head_width=1, head_length=1,color='green')


    # plotting obstacles
    for i in range(len(box)):
        xmin = box[i][0]
        ymin = box[i][1]
        xmax = box[i][2]
        ymax = box[i][3]
        width = xmax - xmin
        height = ymax - ymin
        rect = plt.Rectangle((xmin, ymin), width, height, linewidth=1, edgecolor='k', facecolor='r')
        plt.gca().add_patch(rect)
    plt.pause(2)

    for i in range(len(open)):
        plt.plot(open[i][0],open[i][1],'.')
        plt.pause(0.001)

    # plot Hybrid Astar path
    for i in range(len(path_astar)-1):
        x_curve, y_curve = ([path_astar[i][0],path_astar[i+1][0]],[path_astar[i][1],path_astar[i+1][1]])
        plt.plot(x_curve,y_curve,'b',linewidth=4)
        plt.pause(0.001)

    # plot Dubins path
    for i in range(len(path_dub)-1):
        x_curve, y_curve = ([path_dub[i][0],path_dub[i+1][0]],[path_dub[i][1],path_dub[i+1][1]])
        plt.plot(x_curve,y_curve,'g',linewidth=4)
        plt.pause(0.1)

    # plot car
    for i in range(len(total_path)):
        x = total_path[i][0]
        y = total_path[i][1]
        th = total_path[i][2]
        x_corners, y_corners = plot_car(x,y,th)
        plt.plot(x_corners + [x_corners[0]], y_corners + [y_corners[0]],'k')
        plt.pause(0.001)
        # plt.show()

    plt.show()


if __name__== main():
    main()
    
