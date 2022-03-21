from maze import Maze
import numpy as np
import argparse

a = Maze(verbose=False,step=1,radius=10)

start=(16,16,0)
goal=(300,100,2)

if (a.solve_maze(start,goal)):
    path = a.back_track()    
    a.plot_all_nodes()