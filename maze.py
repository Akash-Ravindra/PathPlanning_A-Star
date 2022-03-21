# import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import math
from queue import PriorityQueue
import tkinter
import pygame
from sympy import true

'''
Custom data type to store each node
'''
class Node:
    def __init__(self,position = [0,0,0], parent = None, cost = np.inf, is_visited = False,hue = np.inf):
        self.__attrdict = {"position":np.array(position),"parent":np.array(parent),"cost":cost,"is_visited":is_visited,"heuristic":hue }

    '''set the position of the state'''
    def set_position(self,position = np.zeros([1,3])):
        self.__attrdict['position'] = np.array(position)
    '''Get the position of the current node'''
    def get_position(self):
        return self.__attrdict['position'] 

    def set_is_visited(self,is_visited = True):
        self.__attrdict['is_visited'] = is_visited
    '''Get the position of the current node'''
    def get_is_visited(self):
        return self.__attrdict['is_visited'] 

    def set_node_parameters(self,dict):
        for keys in dict.keys():
            self.__attrdict[keys] = dict[keys]
    '''Get the index of the parent'''
    def get_parent(self):
        return self.__attrdict['parent']
    '''Set the index of the parent of the current state in the all_state list'''
    def set_parent(self, parent):
        self.__attrdict['parent'] = parent

    '''set the index of the current state'''
    def set_cost(self,cost):
        self.__attrdict['cost'] = cost
    def get_cost(self):
        return self.__attrdict['cost']
    
    
    def set_heuristic(self,hue):
        self.__attrdict['heuristic'] = hue
    def get_heuristic(self):
        return self.__attrdict['heuristic']
    
    def calculate_heuristic(self,coords,set = True): 
        ## Euclidian Hue
        dist = np.linalg.norm(self.__attrdict['position'][:-1]-np.asarray(coords)[:-1])
        dist = dist if dist>1.5/Maze.thresh_xy else 0
        ## Add goal hue
        dist = dist+np.linalg.norm(self.__attrdict['position'][-1]-np.asarray(coords)[-1])
        if set:
            self.set_heuristic(dist)
            return dist
        else:
            return dist
     
    ## Return the angle of the node as a whole of 
    def get_orientation(self):
        return self.__attrdict['position'][-1]*30
    ## Return the angle of the node as the plane number
    def get_angle(self):
        return self.__attrdict['position'][-1]
    
    def get_total_cost(self):
        # if self.__attrdict['heuristic']==np.inf:
        #     return self.__attrdict['cost']
        return self.get_heuristic()+self.get_cost()
    
    
    '''Get cartisian coordinates of each node'''
    def get_cartisian(self):
        x,y,theta = self.get_position()
        x,y = y,x
        return np.array((x*Maze.thresh_xy,((Maze.lim_x*(1/Maze.thresh_xy))-y)*Maze.thresh_xy))


    '''Overload the == operator to compare two states'''
    def __eq__(self, node):
        return np.equal(self.get_position(),node.get_position()).all()
    '''Overload the str to print the object'''
    def __str__(self) -> str:
        return "Self: "+str(self.get_cartisian())+" Angle-"+str(self.get_orientation())+" Parent: "+str(self.get_parent())+" Cost: "+str(self.get_cost())
    '''Overload the str to print the object'''
    def __repr__(self) -> str:
        return (str(self.get_cartisian())+" Angle-"+str(self.get_orientation()))+'\n'
    def __gt__(self,value):
        return (self.get_total_cost()>value)
    def __lt__(self,value):
        return (self.get_total_cost()<value)


class Maze:
    action_set = {'-60':-2,'-30':-1,\
                  '0':0,'30':1,\
                  '60':2} ##action set dict, used to append add to the current index
    lim_x = 250## xlimit in terms of array
    lim_y = 400## Y limit in terms of array
    thresh_xy = 0.5
    anime_fig=None
    fig,ax = None,None
    scatter=None
    def __init__(self,padding = 5,radius = 0,cost:dict = {'-60':1,'-30':1,'0':1,'30':1,'60':1},verbose=True, step = 1) -> None:
        self.__cost = cost              ## Cost for each action
        self.__padding = padding+radius ##padding for the robot
        self.__open_list = PriorityQueue()           ##Open list containing the accessable nodes
        self.__close_list = []          ##closed list containing the visited nodes
        self.path = []
        self.start_goal = []
        self.verbose = verbose
        self.step_size = step
        print(('-'*50)+"\n\t\tInitializing Maze\n"+('-'*50))
        self.__maze = np.array([[[Node([x,y,th])for th in range(12)]
                                 for y in range(int((Maze.lim_y*(1/Maze.thresh_xy))+1))]
                                for x in range(int((Maze.lim_x*(1/Maze.thresh_xy))+1))]
                               ,dtype=Node) ##Empty np array with X*Y Node objects
        ## Update the maze array with the presence of the obstacles
        
        self.update_maze_arrow()
        self.update_maze_circle()
        self.update_maze_hexagon()
        # '''np.save("maze",abc,allow_pickle=True)'''
        # return
        # self.__maze = np.load('../maze.npy',allow_pickle=True)

    def solve_maze(self,start,goal):   
        ## Convert the cartisian coordinates to array frame
        start = self.cartisian_to_idx(list(start)) + (start[-1],)
        goal = self.cartisian_to_idx(list(goal))+ (goal[-1],)
        self.start_goal.append(start)
        self.start_goal.append(goal)
        
        ## Check if the start and goal are accessable
        if(not self.idx_in_maze(self.start_goal[0])):
            print("Start Node inside a Obstacle or out of bounds")
            return False
        if(not self.idx_in_maze(self.start_goal[-1])):
            print("Goal Node inside a Obstacle or out of bounds")
            return False
        
        
        print(('-'*50)+"\n\t\tStarting search\n"+('-'*50))
        ## Set the cost of the start node to zero
        self.__maze[start].set_cost(0)
        ## Apppend the start node and end node into list
        self.__open_list.put(self.__maze[start])
        self.__open_list.put(self.__maze[goal])
        ##sort list, look around, pop open list and append to closed list
        
        while True:
            ## Check if there are still nodes to traverse
            if(self.__open_list.empty()):
                print("Queue empty - no more nodes to explore. Stoping search")
                break
            # look around the first node in the open list NoI = NodeOfInterest
            NoI = self.__open_list.get()
            NoI.set_is_visited()
            
            ## Check for finished condition
            if(self.__maze[self.start_goal[-1]].get_cost()<NoI.get_cost() or NoI.get_heuristic()==0):
                self.new_goal = NoI
                self.start_goal[-1] = tuple(NoI.get_position().tolist())
                print("Found the shortest path to ",self.__maze[self.start_goal[-1]].get_cartisian())
                break
            
            if(self.verbose):
                print(NoI)
            
            # self.look_around(NoI.get_position())
            self.perform_actions(NoI, NoI.get_orientation())
            # Add the first node to the closed list and pop it from open list
            self.__close_list.append(NoI)
            
            
            
        return True
    
    
    ## Back track from the goal to the start node to find the path the robot needs to take
    def back_track(self):
        self.path.clear()
        
        ## Check if the goal was reached
        if(self.__maze[self.start_goal[-1]].get_cost()==np.inf or self.__maze[self.start_goal[-1]].get_parent() is None):
            print("No path from Start to goal")
            return False
        
        print("\n""\n""\n"+('-'*50)+"\n\t\tStarting BackTrack\n"+('-'*50)+"\n")
        ## Iteratively access the parents for each child node
        self.path.append(self.__maze[self.start_goal[-1]]) 
        
        while True:
            node = self.path[-1]
            self.path.append(self.__maze[tuple(node.get_parent())])
            if(self.path[-1]==self.__maze[self.start_goal[0]]):
                break
        ## Flip the array as it have the order from goal to start
        self.path.reverse()
        if(self.verbose):
            print(list(map(str,map(Node.get_cartisian,self.path))))
        print(('-'*50)+"\n\t\tBackTrack Complete\n"+('-'*50)+"\n")            
        return self.path
                        
    def perform_actions(self,node:Node, ori):
        ## Find all the actions that are possible from the current node
        actions = ori+(np.array(list(Maze.action_set.values()))*30)
        for action in actions:
            ## Get the next point based on the current point and the action angle
            end_point_idx = self.vec_endpoint(node.get_cartisian(),action,self.step_size)
            ## Calculate the plane based on the action taken
            angle = (action/30 + 12) % 12
            child_node = end_point_idx + (int(angle),)
             ## Check if the point is in the maze
            if(not self.idx_in_maze(child_node)):
                continue
            parent_node = tuple(node.get_position())
            
            ## check if the point is in the point in an obstacle
            if(not self.__maze[child_node].get_is_visited()):
                ## If not then add it to the list
                self.add_to_list(parent_node,child_node,1)
        pass
    ## Gives the index of the point along a specific angle and magnitude from a given point
    def vec_endpoint(self,start_point, angle, magnitude=1):
        x = np.cos(np.deg2rad(angle))*(magnitude*self.thresh_xy) + start_point[0]
        y = np.sin(np.deg2rad(angle))*(magnitude*self.thresh_xy)+ start_point[1]
        # print(start_point,self.cartisian_cleaner((x,y)),angle)
        
        ### NEED TO CHeck
        if(x<0 or y<0 or x>Maze.lim_y/self.thresh_xy or y > Maze.lim_x/self.thresh_xy):
            return (-1,-1)
        
        return self.cartisian_to_idx((x,y))
        pass
    
    ##Look around the POI
    '''
    def look_around(self,points:np.ndarray):
        #itr through all the possible actions
        actions = tuple(points+np.array(list(Maze.action_set.values())))
        for action,key in zip(actions,Maze.action_set.keys()):
            ## Check if the idx is with in maze size
            if action[0]>=0 and action[0]<=Maze.lim_x and action[1]>=0 and action[1]<=Maze.lim_y:
                ## Check if that element is a Node obj or a None(meaning it is an obstacle)
                if(self.__maze[action[0],action[1]] and not self.__maze[action[0],action[1]].get_is_visited()):
                    ### check if the node is an obstacle
                    self.add_to_list(points,action,self.__cost[key])
                    ### First time coming here, check if it is an obstacle
    '''
        
    ## Add to open list and update node parameters
    def add_to_list(self,parent,child,cost):
        ## If the cost is lesser than the existing cost 
        child = tuple(child)
        parent = tuple(parent)
        if((self.__maze[parent].get_cost()+cost)<self.__maze[child].get_cost()):
            self.__maze[child].calculate_heuristic(self.start_goal[-1])
            ## update the cost with the new cost
            self.__maze[child].set_cost(cost+self.__maze[parent].get_cost())
            ## Set the new parent of the node
            self.__maze[child].set_parent(parent)
            ## Set its visited
            # self.__maze[child].set_is_visited()
            ## Add the node to the open list
            self.__open_list.put(self.__maze[child])
            # print("open list : ",self.__open_list,list(map(Node.get_cost,self.__open_list)))
        
    ## getter for open list
    def get_open_list(self):
        return self.__open_list
    ## getter for close list
    def get_close_list(self):
        return np.array(self.__close_list)
    def get_maze(self):
        return self.__maze
    
    ## Check if a given point is in obstacle, idx is the index
    def point_not_in_obstacle(self,idx):
        ## if the point is in circle or arrow or hexagon return false
        point = self.idx_to_cartisian(idx)
        if((self._is_in_circle(point) or self._is_in_arrow(point) or self._is_in_hexagon(point))):
            self.__maze[idx] = None
            return False
        ## set the obstacle ness of the node
        return True
    
    ## HELPER, Converts idx to cartisian
    def idx_to_cartisian(self, xy):
        xy = list(xy)
        xy[0],xy[1] = xy[1],xy[0]
        xy[1] = abs(xy[1]-(Maze.lim_x*(1/Maze.thresh_xy)))
        return (xy[0]*Maze.thresh_xy,xy[1]*Maze.thresh_xy)
    
    def cartisian_cleaner(self,xy):
        xy = list(xy)
        return ((math.ceil(xy[0]*2))/2.0,(math.ceil(xy[1]*2))/2.0 )
    
    ## HELPER, Converts cartisian to idx
    def cartisian_to_idx(self, xy):
        xy = list(self.cartisian_cleaner(xy))
        xy[0],xy[1] = xy[1],xy[0]
        xy[0] = (Maze.lim_x - xy[0])*(1/Maze.thresh_xy)
        return (int(xy[0]),int(xy[1]*(1/Maze.thresh_xy)))
    ## HELPER, converts cartisian to tkinter coordinates, top left corner is 0,0 , top right corner is 400,0
    def cartisian_to_game(self, xy):
        xy = list(xy)
        xy[1] = Maze.lim_x-xy[1]
        return (xy[0],xy[1])
    
    ## Defines a circle and checks if the idx is within the given circle
    def _is_in_circle(self,idx,radius=40,center=(300,185)):
        x,y = idx
        c1 = ((x-center[0])**2 + (y-center[1])**2 - radius **2)<=0
        if(c1):
            return True
        else:
            return False
    
    ## Defines a arrow and checks if the idx is within the given arrow
    def _is_in_arrow(self,idx):
        x,y = idx
        l1 = (y-(0.31645569620253167*x)-173.60759493670886)<=0
        l2 = (y+(1.2318840579710144*x)-229.3478260869565)>=0
        l3 = (y+(3.2*x)-436)<=0
        l4 = (y-(0.8571428571428571*x)-111.42857142857143)>=0
        if(l3 and l2 and l1):
            return True
        elif((not l3) and l1 and l2 and l4):
            return True
        else:
            return False
            
    
    ## Defines a hexagon and checks if the idx is within the given hexagon
    def _is_in_hexagon(self,idx):
        x,y = idx
        l1 = (y+(0.5792142857142858*x)-175.36285714285717)>=0
        v1 = x>=165
        l2 = (y-(0.5773571428571428*x)-24.94357142857143)<=0
        l3 = (y+(0.5775714285714283*x)-255.92928571428564)<=0
        v2 = x<=235
        l4 = (y-(0.5775000000000001*x)+55.92000000000003)>=0
        if(l1 and l2 and l3 and l4 and v1 and v2):
            return True
        else:
            
            return False
    ## Check each element of the maze that falls in the bounding box of the obstacle 
    def update_maze_circle(self,radius=40,center=(300,185)):
        ## Bounding box
        coords = [np.array(center)-self.__padding-radius,np.array(center)+self.__padding+radius]
        start = ((math.floor(coords[0][0]),math.floor(coords[0][1])))
        end = ((math.ceil(coords[1][0]),math.ceil(coords[1][1])))
        ## Permutation of coordinates
        x,y = np.ogrid[start[0]:end[0],start[1]:end[1]]
        
        x = x.T
        padding = int(self.__padding)
        xx,yy = np.ogrid[:(Maze.lim_x*(1/Maze.thresh_xy))+1,:(Maze.lim_y*(1/Maze.thresh_xy))+1]
        ## Remove nodes that are in the obstacle +padding in all directions
        for yi in y[0]:
            for xi in x[0]:
                if self._is_in_circle((xi,yi)):
                    idx,idy = self.cartisian_to_idx((xi,yi))
                    mask = (xx-idx)**2 + (yy-idy)**2 - (padding*(1/Maze.thresh_xy))**2<=0
                    self.__maze[mask,:] = None 
        pass
    
    ## Check each element of the maze that falls in the bounding box of the obstacle 
    def update_maze_arrow(self):
        points = np.array([(115,210),(80,180),(105,100),(36,185)])
        coords = [points[:,0].min()-self.__padding,points[:,1].min()-self.__padding,points[:,0].max()+self.__padding,points[:,1].max()+self.__padding]
        start = ((math.floor(coords[0]),math.floor(coords[1])))
        end = ((math.ceil(coords[2]),math.ceil(coords[3])))
        
        x,y = np.ogrid[start[0]:end[0],start[1]:end[1]]
        
        x = x.T
        padding = int(self.__padding)
        xx,yy = np.ogrid[:(Maze.lim_x*(1/Maze.thresh_xy))+1,:(Maze.lim_y*(1/Maze.thresh_xy))+1]
        # Remove nodes that are in the obstacle +padding in all directions
        for yi in y[0]:
            for xi in x[0]:
                if self._is_in_arrow((xi,yi)):
                    idx,idy = self.cartisian_to_idx((xi,yi))
                    mask = (xx-idx)**2 + (yy-idy)**2 - (padding*(1/Maze.thresh_xy))**2<=0
                    self.__maze[mask,:] = None 
        pass
    ## Check each element of the maze that falls in the bounding box of the obstacle 
    def update_maze_hexagon(self):
        points = np.array([(200,59.58),(165,79.7925),(165,120.2075),(200,140.415),(235,120.2075),(235,79.7925)])
        coords = [points[:,0].min()-self.__padding,points[:,1].min()-self.__padding,points[:,0].max()+self.__padding,points[:,1].max()+self.__padding]
        start = ((math.floor(coords[0]),math.floor(coords[1])))
        end = ((math.ceil(coords[2]),math.ceil(coords[3])))
        
        x,y = np.ogrid[start[0]:end[0],start[1]:end[1]]
        
        x = x.T
        padding = int(self.__padding)
        xx,yy = np.ogrid[:(Maze.lim_x*(1/Maze.thresh_xy))+1,:(Maze.lim_y*(1/Maze.thresh_xy))+1]
        # Remove nodes that are in the obstacle +padding in all directions
        for yi in y[0]:
            for xi in x[0]:
                if self._is_in_hexagon((xi,yi)):
                    idx,idy = self.cartisian_to_idx((xi,yi))
                    mask = (xx-idx)**2 + (yy-idy)**2 - (padding*(1/Maze.thresh_xy))**2<=0
                    self.__maze[mask,:] = None 
        pass
    def idx_in_maze(self, coords):
        if (coords[0]<0+self.__padding/Maze.thresh_xy or coords[0]>((Maze.lim_x-self.__padding)/self.thresh_xy) or\
            coords[1]<0+self.__padding/Maze.thresh_xy or coords[1]>((Maze.lim_y-self.__padding)/self.thresh_xy) or\
                coords[2]<0 or coords[2]>11 or\
                    type(self.__maze[coords]) == type(None)):
                return False
        else:
            return True
    # Plot the completed path as an animation
    def game_plot(self):
        ## Define the size of the window
        window_width = 400
        window_height = 250
        ## The root object
        Window = tkinter.Tk()
        Window.title("Maze Path Visualization")
        Window.geometry('400x250')
        ## The main canvas object where all the objects are drawn
        canvas = tkinter.Canvas(Window)
        canvas.configure(bg="Black")
        ## Refresh rate
        animation_refresh_seconds = 0.00001
        canvas.pack(fill="both", expand=True)
        ## The image element used to update the searched nodes
        img = tkinter.PhotoImage(width=window_width,height=window_height)
        canvas.create_image((window_width/2,window_height/2),image = img)
        p1,p2,p3,p4 = self.cartisian_to_game([115,210]),\
                        self.cartisian_to_game([80,180]),\
                        self.cartisian_to_game([105,100]),\
                        self.cartisian_to_game([36,185]) ##Arrow Coordinates
        canvas.create_polygon(p1,p2,p3,p4,fill='Red', outline='Blue',width=1) ##Arrow 
        p1,p2,p3,p4,p5,p6 = self.cartisian_to_game([200,59.58]),\
                            self.cartisian_to_game([165,79.7925]),\
                            self.cartisian_to_game([165,120.2075]),\
                            self.cartisian_to_game([200,140.415]),\
                            self.cartisian_to_game([235,120.2075]),\
                            self.cartisian_to_game([235,79.7925])
        canvas.create_polygon(p1,p2,p3,p4,p5,p6,fill='Red', outline='Blue',width=1)#Hexagon
        p1,radius = self.cartisian_to_game([300,185]),40 # circle
        x,y = p1
        canvas.create_oval(x-radius,y-radius,x+radius,y+radius,fill='Red', outline='Blue',width=0.5)#circle
        ## For each element of closed list, color the image, this is the order in which they were explored
        for i in range(len(self.__close_list)-1):
            img.put("#ffffff",(self.cartisian_to_game(self.__close_list[i].get_cartisian())))
            Window.update()
            time.sleep(animation_refresh_seconds)
        ## Draw the path of the found by the algorithm
        animation_refresh_seconds = 0.001
        start = self.cartisian_to_game(self.path[0].get_cartisian())
        goal = self.cartisian_to_game(self.path[-1].get_cartisian())
        radius = 5
        ## Circles to define the start and end node
        canvas.create_oval(start[0]-radius,start[1]-radius,start[0]+radius,start[1]+radius, fill = 'Orange')
        canvas.create_oval(goal[0]-radius,goal[1]-radius,goal[0]+radius,goal[1]+radius, fill = 'Green')
        ## Iterate over all the nodes in the path and color them in
        for i in range(len(self.path)):
            img.put("#00ff00",(self.cartisian_to_game(self.path[i].get_cartisian())))
            Window.update()
            time.sleep(animation_refresh_seconds)
        time.sleep(5)
        Window.destroy()
    # A simple scatter plot to see the path taken by the robot
    # def cv_plot(self):
    #     background = np.zeros((250,400),np.uint8)
    #     arrow = np.array([[115,210],[80,180],[105,100],[36,185]],np.int32)
    #     arrow[:,0] = 250-arrow[:,0]
    #     arrow[:,0],arrow[:,1] = arrow[:,1],arrow[:,0]
    #     arrow = arrow.reshape((-1,1,2))
    #     background = cv.polylines(background,[arrow], True,255,10)
    #     cv.imshow("Maze",background)
    #     cv.waitKey()
    #     cv.destroyAllWindows()
    def simple_plot_path(self):
        ## Create the obstacles
        arrow = patch.Polygon([[115,210],[80,180],[105,100],[36,185]],color="red")
        circle = patch.Circle([300,185],40,color="red")
        Hexagon = patch.Polygon([(200,59.58),(165,79.7925),(165,120.2075),(200,140.415),(235,120.2075),(235,79.7925)],color="red")
        ## Add the patches to the ax
        fig,ax = plt.subplots()
        ax.set_facecolor((0,0,0))
        ax.add_patch(arrow)
        ax.add_patch(circle)
        ax.add_patch(Hexagon)
        fig.canvas.toolbar_visible = False
        fig.canvas.header_visible = False
        fig.canvas.footer_visible = False
        ax.grid(False)
        plt.xlim((0,Maze.lim_y))
        plt.ylim((0,Maze.lim_x))
        # plot each of the points on the graph
        for i in self.path:
            x,y = i.get_cartisian()
            ax.scatter(x,y,s=1,marker="s",linewidths=0.25,edgecolors=[0,0,0], color = 'blue')
        plt.title("Maze path using A-star")
        plt.show(block=False)
        plt.pause(5)
        plt.close()
    def plot_boilerplate(self):
         ## Create the obstacles
        arrow = patch.Polygon([[115,210],[80,180],[105,100],[36,185]],color="red")
        circle = patch.Circle([300,185],40,color="red")
        Hexagon = patch.Polygon([(200,59.58),(165,79.7925),(165,120.2075),(200,140.415),(235,120.2075),(235,79.7925)],color="red")
        ## Add the patches to the ax
        fig,ax = plt.subplots()
        ax.set_facecolor((0,0,0))
        ax.add_patch(arrow)
        ax.add_patch(circle)
        ax.add_patch(Hexagon)
        fig.canvas.toolbar_visible = False
        fig.canvas.header_visible = False
        fig.canvas.footer_visible = False
        ax.grid(False)
        plt.xlim((0,Maze.lim_y))
        plt.ylim((0,Maze.lim_x))
        return fig,ax
    def full_plot_path(self):
        ## Create the obstacles
        fig,ax = self.plot_boilerplate()
        # plot each of the points on the graph
        for i in range(len(self.path)-1):
    
            x,y = self.path[i].get_cartisian()
            u,v = self.path[i+1].get_cartisian() 
            #ax.scatter(x,y,s=1,marker="s",linewidths=0.25,edgecolors=[0,0,0], color = 'blue')
            plt.quiver(x, y, u-x, v-y, units='xy', scale =0.25,  color= 'g', headwidth = 1, headlength=0)
        plt.title("Maze path using  A-star")
        plt.show(block=False)
        plt.pause(5)
        plt.close()
        
    def plot_explored_nodes(self):
        # closed_list = self.get_close_list()
        closed_list = self.path
        fig,ax = self.plot_boilerplate()
        for node in closed_list[1:]:
            parent = self.__maze[node.get_parent()].get_cartisian()
            child = node.get_cartisian()
            plt.plot((parent[0],child[0]),(parent[1],child[1]), color= 'g', linewidth = 1)
            point = plt.Circle((child[0],child[1]),0.5,color = 'w')
            ax.add_patch(point)
            pass
        pass
        
        
