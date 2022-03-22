from maze import Maze
import numpy as np
import argparse

def get_inputs():
    parser = argparse.ArgumentParser(description='Finds the path from start to goal using A-star algorithm',usage='\t\t\tpython %(prog)s  -xs 20 -ys 15 -ths 150 -xg 300 -yg 120 -thg 180 -stp 5\t\t\tSilent Operation\n\t\t\tpython %(prog)s -xs 20 -ys 15 -ths 150 -xg 300 -yg 120 -thg 180 -stp 5 --output\t\tVerbose Operation ',)
    parser.add_argument('-xs', type=np.float16, default=15, help="The x coordinate of the Start Node")
    parser.add_argument('-ys', type=np.float16, default=15, help="The y coordinate of the Start Node")
    parser.add_argument('-ths', type=np.float16, default=0, help="The theta value signifying the orientation at Start Node")
    parser.add_argument('-xg', type=np.float16, default=385, help="The x coordinate of the Goal Node")
    parser.add_argument('-yg', type=np.float16, default=235, help="The y coordinate of the Goal Node")
    parser.add_argument('-thg', type=np.float16, default=0, help="The theta value signifying the orientation at Goal Node")
    parser.add_argument('-stp', type=np.float16, default=5, help="The minimum step size the robot can take")
    parser.add_argument('-padding', type=np.uint8, default=5, help="The minimum distance from boundaries and obstacles")
    parser.add_argument('-r', type=np.uint8, default=10, help="Radius of the robot")
    parser.add_argument('-o', '--output', action='store_true', help="Turns on verbose")
    
    
    args = parser.parse_args()
    start =(args.xs,args.ys,args.ths)
    goal = (args.xg,args.yg,args.thg)
    step = args.stp
    padding = (args.padding, args.r)
    v = args.output
    return [start,goal,step,padding,v]
## Execute the methods of the Maze class

def main(start,goal,step,padding,verbose=False):
    
    a = Maze(verbose=verbose,step=step,radius=padding[-1],padding=padding[0],)
    
    if (a.solve_maze(start,goal)):
        path = a.back_track()
        a.game_display()    
        #a.plot_all_nodes()
    print(('-'*50)+"\n\t\tCLOSING\t\t\n"+('-'*50))
    

if __name__== '__main__':
    start,goal,step,padding,v = get_inputs()
    if(1>step>10 or padding[0]<0 or padding[1]<0):
        print("Inputs are not valid")
    else:
        main(start=start,goal=goal,step=step,padding=padding,verbose=v)
    





