

# PathPlanning_A-Star

## Requirements
- tkinter - tk=8.6.10=hbc83047_0
- numpy
- matplotlib
- pygame

---
## Instructions 
-    To run the file simply run the command 
```shell
    python astar.py
```
 -   The above command will run the file with default start and goal nodes, that is, (15,15) and (385, 235).

To run the robot from a desired location, use the following commnd
```shell
    python main.py -xs 20 -ys 15 -ths 5 -xg 300 -yg 120 -thg 6 -stp 2 -o 
```
- Where xs,ys,ths and xg,yg,thg are integer values of the x,y and theta coordinates of start and goal node respectively. 

- stp is the minimum step size the robot can take, -o or --output turns on verbose.

- To Exit the display, press escape key.
 
For further information:
```bash
    python astar.py --help
```

## Github link
[Github](https://github.com/Akash-Ravindra/PathPlanning_A-Star/tree/threeD)