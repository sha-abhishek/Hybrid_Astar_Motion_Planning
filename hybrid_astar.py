from utils import *
from grid import Grid
from dubins import *

def hybrid_astar(grid_dim,cell_size,start_conf,goal_conf,car,obs):
    """
    Caclulates trajectory using Astar Algorithm and dubins curves with 
    given start and goal configuration
    """
    open_list = PriorityQueue(order=min, f=lambda v: v.f)
    closed_list = OrderedSet()
    init_node = start_conf
    cur_node = init_node
    grid_env = Grid(grid_dim,cell_size)
    grid_discr = grid_env.make_grid()
    
    
    start_conf_discr = discr_cor(start_conf,cell_size) 
    goal_conf_discr = discr_cor(goal_conf,cell_size)

    reached_goal = False
    open = []
    closed = []

    h =  np.sqrt((goal_conf[0]-start_conf[0])**2 + (goal_conf[1]-start_conf[1])**2) # eucl dist
    g = 0
    f = g+h
    grid_discr[start_conf_discr[0]][start_conf_discr[1]] = (start_conf,f,None)  #(config,f value, parent conf)
    
    open_list.put(init_node, Value(f=f,g=g))

    open.append(init_node)
    
    while open_list.__len__() > 0:

        node,val = open_list.pop()
        node_discr = discr_cor(node,cell_size)
        closed.append(node[:2])
        
        if node_discr == goal_conf_discr:
            closed_list.add(node_discr)    # closed list is list of discrete closed nodes 
            reached_goal = True
            print('goal reached')
            break
        closed_list.add(node_discr)

        next_confs = car.astar_step(node)    
        next_confs = valid_config(next_confs, grid_dim)

        safe_confs = []
        if len(next_confs)>0:
            for i in range(len(next_confs)):
                if aabb_col(next_confs[i],obs):
                    continue
                else:
                    safe_confs.append(next_confs[i])

        for i in range(len(safe_confs)):
            safe_conf_disc = discr_cor(safe_confs[i],cell_size)
            sc_d_x = safe_conf_disc[0]
            sc_d_y = safe_conf_disc[1]

            if safe_conf_disc not in closed_list._container:

                sc_x = safe_confs[i][0]
                sc_y = safe_confs[i][1]
                sc_th = safe_confs[i][2]
                if sc_th != node[2]:
                    st_c = 1.5
                else:
                    st_c = 1
                
                

                sc_g = val.g + st_c
                sc_h = np.sqrt((goal_conf[0]-sc_x)**2 + (goal_conf[1]-sc_y)**2)
                
                sc_f = sc_g + sc_h 

                if sc_h < 1*h:
                    dub_path, dub_len = dubin_path(safe_confs[i],goal_conf)
                    dub_valid = valid_config(dub_path,grid_dim)
                    col_check = False
                    if dub_len == len(dub_valid):
                        for j in range(len(dub_path)):
                            col_check = aabb_col(dub_path[j], obs)
                            if col_check:
                                break

                        if not col_check:
                            path = []
                            reached_goal = True
                            goal_conf = safe_confs[i]
                            goal_conf_discr = discr_cor(goal_conf)
                            path2 = dub_path

                            last_node = grid_discr[node_discr[0]][node_discr[1]][0]
                            while discr_cor(last_node,cell_size) != start_conf_discr:
                                last_node_discr = discr_cor(last_node,cell_size)
                                parent_node = grid_discr[last_node_discr[0]][last_node_discr[1]][2]
                                path.insert(0,last_node)
                                last_node = parent_node
                            
                            return path, path2, open
                
                if sc_d_x < len(grid_discr) and sc_d_y < len(grid_discr[0]): 
                    if grid_discr[sc_d_x][sc_d_y] != 0:
                        if sc_f < grid_discr[sc_d_x][sc_d_y][1]:
                            grid_discr[sc_d_x][sc_d_y] = (safe_confs[i],f,node) #(config,f value, parent conf)
                            
                    else:
                        open_list.put(safe_confs[i], Value(f=sc_f,g=sc_g))
                        grid_discr[sc_d_x][sc_d_y] = (safe_confs[i],f,node) #(config,f value, parent conf)
                        open.append(safe_confs[i])

    
    path = []

    if reached_goal:
        last_node = grid_discr[goal_conf_discr[0]][goal_conf_discr[1]][0]
        while discr_cor(last_node,cell_size) != start_conf_discr:
            last_node_discr = discr_cor(last_node,cell_size)
            parent_node = grid_discr[last_node_discr[0]][last_node_discr[1]][2]
            path.insert(0,last_node)
            last_node = parent_node

    return path, open
