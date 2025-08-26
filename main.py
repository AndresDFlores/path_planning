import pandas as pd
from path_planner import *


# import obstacle map:  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
for sheet_num in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]:

    print(f'SHEET{sheet_num}')
    
    #  load map
    map = pd.read_excel(r'maps.xlsx', sheet_name=f'Sheet{sheet_num}')
    map_dims = map.shape


    #  initialize class
    binary_map = [map.iloc[idx].tolist() for idx in range(map.shape[0])]
    path_planner_class = PathPlanner(binary_map)


    # print messages and export scoring data
    literal = False
    path_planner_class.set_literal(literal)


    #  run path planning algorithm
    path_planner_class.main()
    path_planner_class.plot_path()



    #  --- DEV export cells

    if literal:

        df = path_planner_class.get_isolated_key_view(dict_key='h')
        df1 = pd.DataFrame(df)

        df = path_planner_class.get_isolated_key_view(dict_key='g')
        df2 = pd.DataFrame(df)

        df = path_planner_class.get_isolated_key_view(dict_key='f')
        df3 = pd.DataFrame(df)

        df = path_planner_class.get_isolated_key_view(dict_key='cell_global_coord')
        df4 = pd.DataFrame(df)

        with pd.ExcelWriter('score_map.xlsx') as writer:  
            df1.to_excel(writer, sheet_name='h')
            df2.to_excel(writer, sheet_name='g')
            df3.to_excel(writer, sheet_name='f')
            df4.to_excel(writer, sheet_name='coords')
