

# PathPlanning_A-Star
***Note - The animation in the video file attached starts from the 45th second. Please be patient.***

## Requirements
- tkinter - tk=8.6.10=hbc83047_0
- numpy
- matplotlib
- pygame

---
## Instructions 
- To run the file simply run the command 
    ```shell
    python astar.py
    ```
 -   The above command will run the file with default start and goal nodes, that is, (15,15,0) and (385, 235,0) with a step size of 5.

- To run the robot from a desired location, use the following commnd
    ```shell
    python astar.py -xs 20 -ys 15 -ths 150 -xg 300 -yg 120 -thg 180 -stp 5 -o 
    ```
- Where (xs,ys,ths) and (xg,yg,thg) are positive integer values of the x,y and theta coordinates of start and goal node respectively. All values of theta would be floored to the nearest multiple of 30

- stp is the minimum step size the robot can take(1<=L<=10)
- -o or --output turns on verbose.

- To Exit the display, press escape key.
 
For further information:
```bash
    python astar.py --help
```
____
## Github link
[Github](https://github.com/Akash-Ravindra/PathPlanning_A-Star/tree/main)