from a_star import *


class PathPlanner(AStar):

    @classmethod
    def set_literal(cls, literal=False):
        cls.literal=literal



    def __init__(self, binary_map):

        #  initialize A* functions class
        self.a_star_class = AStar(binary_map)

        #  init literal status
        self.set_literal(literal=False)



    def main(self):

        #  set literal status in a_star_class
        self.a_star_class.set_literal(self.literal)

        
        # interpret global map and identify key locations
        self.a_star_class.get_flag_indices()  # 255: origin | -255: destination


        #  init global score map (MAY DELETE)
        global_score_map = self.a_star_class.init_score_map()
        self.a_star_class.update_global_score_map(global_score_map)


        #  initialize variables
        iter = 0
        focus_node = self.a_star_class.flag_indices[255]  # this is the start cell index coordinate



        #  ---  ALGORITHM  ---
        while True:
        
            #  'close' the focus cell
            self.a_star_class.global_score_map[focus_node[0]][focus_node[-1]]['cell_open'] = False

            
            #  confirm that the focus cell is either open (0) or destination (-255)
            focus_node_val = global_score_map[focus_node[0]][focus_node[-1]]['cell_val']
            

            #  break out of algorithm if focus_node is the destination flag
            if focus_node_val == -255:
                break


            if self.literal: print(f'\n\nITER: {iter}\nFOCUS: {focus_node}\n')


            #  relate the nxn cost kernel to the global map cells' coordinates
            kernel_coords = self.a_star_class.get_kernel_coords(cell=focus_node)


            #  --- iterate through kernel
            for kernel_row_idx, kernel_row in enumerate(kernel_coords):
                for kernel_col_idx, _ in enumerate(kernel_row):


                    #  get global map cell coordinates associated from kernel cell
                    global_node_coord = kernel_coords[kernel_row_idx][kernel_col_idx]


                    #  get global map cell value (0, -1, 255, -255) for kernel cells
                    global_map_node = self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]
                    global_map_node_val = global_map_node['cell_val']


                    #  skip inaccessible cells from scoring
                    if global_map_node_val==1:
                        continue


                    #  distance from neighbor cell to origin cell
                    g_score = self.a_star_class.get_g_score(
                        current_pos=global_node_coord, 
                        focus_node_pos=focus_node)
                    

                    #  distance from neighbor cell to destination cell
                    h_score = self.a_star_class.get_h_score(
                        current_pos=global_node_coord, 
                        flag_indices=self.a_star_class.flag_indices)
                    
                    
                    #  f = g+h
                    f_score = g_score+h_score


                    if self.literal:
                        print(f'\tKernel Coord: {global_node_coord}')
                        print(f'\tNode Value: {global_map_node_val}')
                        print(f'\tG: {g_score}')
                        print(f'\tH: {h_score}')
                        print(f'\tF: {f_score}')
                    

                    #  skip cell if it is already closed
                    if global_map_node['cell_open'] is False:
                        continue


                    #  skip cell if it was previously explored and it already had a lower g_score
                    if global_map_node['cell_explored']:

                        g_prior = self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]['g']
                        if g_prior<=g_score:
                            continue


                    #  record kernel_scores in dictionary
                    self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]['cell_explored']=True
                    self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]['kernel_parent_global_coord']=focus_node
                    self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]['g']=g_score
                    self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]['h']=h_score
                    self.a_star_class.global_score_map[global_node_coord[0]][global_node_coord[-1]]['f']=f_score



            #  --- identify the next focus kernel

            #  isolate only kernel cells on global map
            kernel_global_nodes = self.a_star_class.get_kernel_nodes(kernel_coords)

            #  save the kernel of explored cells built around the focus cell
            self.a_star_class.global_score_map[focus_node[0]][focus_node[-1]]['kernel_neighbors'] = kernel_global_nodes


            #  update open cells queue
            self.a_star_class.update_open_list()


            #  identify open cell on global map with the min F score
            next_node = self.a_star_class.get_open_nodes_min()
            focus_node = next_node['cell_global_coord']


            #  increment iter
            iter+=1



        #  get final path
        self.a_star_class.get_final_path()


    def plot_path(self):

        summary=f'\nARRIVED AT {self.a_star_class.final_path[-1]} IN {len(self.a_star_class.final_path)-1} STEPS\n\n'
        self.a_star_class.plot_map_path(title=summary)
