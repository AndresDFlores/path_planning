import numpy as np
import datetime as dt

from copy import deepcopy

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


class AStar:
    
    @classmethod
    def set_literal(cls, literal=False):
        cls.literal=literal



    def __init__(self, binary_map):

        #  convert map from dataframe into iterable matrix
        self.binary_map = binary_map

        #  define start and end points
        self.flags = [255, -255]

        #  track all open and explored cells
        self.open_nodes=[]

        #  track all open and explored cells
        self.final_path=[]

        #  init literal status
        self.set_literal(literal=False)



    #  this method updates the global_score_map class variable, which characterizes each map cell
    @classmethod
    def update_global_score_map(cls, global_score_map):
        cls.global_score_map=global_score_map



    #  update and sort the list of open cells that have been explored
    def update_open_list(self):

        open_nodes_list = []
        for row in self.global_score_map:
            for cell in row:

                if cell is None:
                    continue

                global_node_coord = cell['cell_global_coord']

                if cell['cell_open']==False:
                    if self.literal: print(f'\nSKIPPED {global_node_coord}:  CELL CLOSED')
                    continue
                if cell['cell_explored']==False:
                    if self.literal: print(f'\nSKIPPED {global_node_coord}:  CELL NOT EXPLORED')
                    continue
                if cell['cell_val']==1:
                    if self.literal: print(f'\nSKIPPED {global_node_coord}:  CELL NOT ACCESSIBLE')
                    continue


                if cell['cell_open'] is False:
                    if self.literal: print(f'\nSKIPPED {global_node_coord}:  CELL PREVIOUSLY TRAVELED')
                    continue
                    
                open_nodes_list.append(cell)
                    
        #  sort open_nodes_list along 'f', then along 'h'
        self.open_nodes = sorted(open_nodes_list, key=lambda x: (x['f'], x['h']))



    #  characterize binary map flags
    def get_flag_indices(self):

        flag_indices=dict()

        #  iterate through each cell index
        for row_idx, row in enumerate(self.binary_map):
            for col_idx, col_val in enumerate(row):

                #  check the current cell for a flag value
                for flag in self.flags:
                    if col_val==flag:
                        flag_indices[flag]=(row_idx,col_idx)

        self.flag_indices = flag_indices



    #  build kernel that isolates the cells immediately surrounding the focus cell
    def get_kernel_coords(self, cell:tuple):

        #  input cell is the focus cell's index coordinates (row, col)

        top_left = (cell[0]-1, cell[-1]-1)
        top = (cell[0]-1, cell[-1])
        top_right = (cell[0]-1, cell[-1]+1)


        left = (cell[0], cell[-1]-1)
        center = (cell[0], cell[-1])
        right = (cell[0], cell[-1]+1)

        
        bottom_left = (cell[0]+1, cell[-1]-1)
        bottom = (cell[0]+1, cell[-1])
        bottom_right = (cell[0]+1, cell[-1]+1)

        
        kernel = [
            [top_left, top, top_right],
            [left, center, right],
            [bottom_left, bottom, bottom_right]
        ]

        return kernel
    


    def init_score_map(self):

        #  g: score from starting cell
        #  h: score from destination cell
        #  f = g + h

        score_map=deepcopy(self.binary_map)

        #  iterate through each global cell index
        for global_row_idx, row in enumerate(self.binary_map):
            for global_col_idx, val in enumerate(row):


                cell_score = dict(
                    cell_global_coord = (global_row_idx, global_col_idx),
                    cell_val=val,
                    cell_open=True,
                    cell_explored=False,
                    kernel_parent_global_coord=None,
                    kernel_neighbors=None,
                    g = 0, 
                    h = 0,
                    f = 0
                    )
                            
                score_map[global_row_idx][global_col_idx]=cell_score

        return score_map
   


    @staticmethod
    def get_euclidean_distance(point_1:tuple, point_2:tuple):

        euclid_dist = np.sqrt((point_1[0]-point_2[0])**2+(point_1[-1]-point_2[-1])**2)

        return euclid_dist
    


    #  distance to origin cell
    def get_g_score(self, current_pos:tuple, focus_node_pos:tuple):

        focus_node_g_score = self.global_score_map[focus_node_pos[0]][focus_node_pos[-1]]['g']

        point_1 = current_pos
        point_2 = focus_node_pos

        #  calculate the equclidean distance between the current position index coords and the origin index coords
        euclid_distance = self.get_euclidean_distance(point_1, point_2)

        #  calculate the g_score
        g_score = int(euclid_distance*10)+focus_node_g_score
        
        return g_score



    #  distance to end cell
    def get_h_score(self, current_pos:tuple, flag_indices:dict):
        #  flag indices are the index coordinates of the flags in the binary map, input as a tuple in the form of (row, col)

        point_1 = current_pos
        point_2 = flag_indices[-255]

        #  calculate the equclidean distance between the current position index coords and the destination index coords
        euclid_distance = self.get_euclidean_distance(point_1, point_2)

        #  calculate the h_score
        h_score = int(euclid_distance*10)
        
        return h_score

    

    #  isolate only kernel cells on global map
    def get_kernel_nodes(self, kernel_coords):

        kernel_scores = []
        for row in kernel_coords:

            row_data = []
            for coord in row:
                val = self.global_score_map[coord[0]][coord[-1]]
                row_data.append(val)

            kernel_scores.append(row_data)

        return kernel_scores
    

    def get_open_nodes_min(self):

        min_node = self.open_nodes.pop(0)

        return min_node


    #  get final path
    def get_final_path(self):

        focus_node = self.flag_indices[-255]
        self.final_path.append(focus_node)
        
        cell_val = self.global_score_map[focus_node[0]][focus_node[-1]]['cell_val']
        while cell_val != 255:

            focus_node = self.global_score_map[focus_node[0]][focus_node[-1]]['kernel_parent_global_coord']
            cell_val = self.global_score_map[focus_node[0]][focus_node[-1]]['cell_val']

            self.final_path.append(focus_node)

        self.final_path.reverse()



# --- DEV TOOLS TO DELETE---


    def get_isolated_key_view(self, dict_key):

        isolated_data = []
        for row in self.global_score_map:
            row_data = [col[dict_key] for col in row]
            isolated_data.append(row_data)

        return isolated_data
         


    def plot_map_path(self, title=''):

        #  init fig
        fig, ax = plt.subplots()

        
        #  only plot map with known obstacles
        map = []
        for row in self.binary_map:
            row_data = [1 if col==1 else 0 for col in row ]
            map.append(row_data)
        

        #  define binary map colors
        cmap = ListedColormap(['grey', 'black'])


        # display the binary map
        ax.imshow(map, cmap)


        #  plot origin and destination
        ax.scatter(self.flag_indices[255][-1], self.flag_indices[255][0], s=100, c='g', label='ORIGIN')
        ax.scatter(self.flag_indices[-255][-1], self.flag_indices[-255][0], s=100, marker='x', c='r', label='DESTINATION')


        #  plot each step in defined path
        for step in range(len(self.final_path)-1):

            #  line bounds
            start_step_row, start_step_col = self.final_path[step]
            stop_step_row, stop_step_col = self.final_path[step+1]


            #  path lines
            x = [start_step_col, stop_step_col]
            y = [start_step_row, stop_step_row]
            ax.plot(x, y, linewidth=0.5, color='yellow')


            #  directional arrows
            u = np.diff(x)
            v = np.diff(y)
            pos_x = x[:-1] + u/2
            pos_y = y[:-1] + v/2
            norm = np.sqrt(u**2+v**2) 
            ax.quiver(pos_x, pos_y, u/norm, v/norm, angles="xy", zorder=3, pivot="mid", color='yellow')


        #  remove axis ticks
        ax.set_xticks([])
        ax.set_yticks([])


        #  other plot formatting
        ax.set_title(title, y=0.9)
        ax.legend(loc='lower right')


        #  save plot
        date_time = dt.datetime.now()
        date_time = date_time.strftime('%Y%m%d_%H%M%S.%f')[:-3]
        fig.savefig(f'path_figs/{date_time}.png')
