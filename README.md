

# PathPlanning_A-Star

## Requirements
- tkinter - tk=8.6.10=hbc83047_0
- numpy
- matplotlib
- sympy
- pygame
- Also included is a requirements.txt for dependencies
---
## Instructions 
-    To run the file simply run the command 
```shell
    python main.py
```
 -   The above command will run the file with default start and goal nodes, that is, (15,15) and (385, 235).

To run the robot from a desired location, use the following commnd
```shell
    python main.py -xs a -ys b -ths c -xg d -yg e -thg f -stp g -padding h -r i -o 
```
Where a,b,c and d,e,f are integer values of the x,y and theta coordinates of start and goal node respectively. 

g is the minimum step size the robot can take, h is the minimum distance from obstacles and boundaries. 

i is the radius of the robot. -o or --output turns on verbose.

For further information:
```bash
    python main.py --help
```

## Github link
[Github](https://github.com/Akash-Ravindra/PathPlanning_A-Star/tree/threeD)