import numpy as np

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'u'
        self.maze_dim = maze_dim
        #map the maze by keeping track of the value of the wall at the current cell
        self.map = [[-1 for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        #used for the counter which counts the number of times the robot has been in this particular cell when mapping.
        self.count = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]


        self.dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
                            'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
                            'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
                            'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}

        self.dir_int = {'u': 1, 'r': 2, 'd': 4, 'l': 8,
                        'up': 1, 'right': 2, 'down': 4, 'left': 8}

        self.action = {'1': 'u', '2': 'r', '4': 'd', '8': 'l'}                

        self.dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
                            'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

        self.dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
                         'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}    

        self.dir_angle = {'0':-90, '1':0, '2':90}                            
        
        self.reset = False
        self.A_star = True

        self.policy = [[(-1, ' ') for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        self.time_steps = 0

        self.path = [[' ' for row in range(self.maze_dim)] for col in range(self.maze_dim)]                                    

    
    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''

        rotation = 0
        movement = 0

        if self.reset == False:
            rotation, movement = self.map_maze(sensors)
        else:
            rotation, movement = self.search_maze(sensors)   

        return rotation, movement


    def map_maze(self, sensors):
        #add one count to the current location
        x = self.location[0]
        y = self.location[1]
        self.count[x][y] += 1
        
        #compute the integer that represents the walls of the current cell
        if self.map[x][y] == -1:
            wall = 0
            for i in range(len(sensors)):
                if sensors[i] != 0:
                    key = self.dir_sensors[self.heading][i]
                    wall += self.dir_int[key]
            if self.location != [0 ,0]:
                wall += self.dir_int[self.dir_reverse[self.heading]] 
            self.map[x][y] = wall

        #if we reach the goal, return 'Reset'
        goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        if x in goal_bounds and y in goal_bounds:

            self.reset = True
            self.location = [0, 0]
            self.heading = 'u'
            return 'Reset', 'Reset'

        #choose next cell to move into based on a combination of the count and the heuristic
        choice_list = []
        for i in range(len(sensors)):
            if sensors[i] != 0:
                cell = [0, 0]
                direction = self.dir_sensors[self.heading][i]
                cell[0] = self.location[0] + self.dir_move[direction][0]
                cell[1] = self.location[1] + self.dir_move[direction][1]
                cost = self.count[cell[0]][cell[1]] + self.heuristic(cell)
                choice_list.append([cost, cell[0], cell[1], i])

        if len(choice_list) != 0:
            choice_list.sort()
            choice_list.reverse()
            choice = choice_list.pop()
            self.location[0] = choice[1]
            self.location[1] = choice[2]

            rotation = self.dir_angle[str(choice[3])]

            if rotation == -90:
                self.heading = self.dir_sensors[self.heading][0]
            elif rotation == 90:
                self.heading = self.dir_sensors[self.heading][2]
            elif rotation == 0:
                pass

            return rotation, 1    

        # in a dead end, rotate 90(or -90) degree and don't move    
        else:
            self.heading = self.dir_sensors[self.heading][2]
            return 90, 0        


    def search_maze(self, sensors):
        if self.A_star:
            self.policy = self.A_star_search()
            self.A_star = False

        init_rotation = -1
        count = 0

        while True:
            x = self.location[0]
            y = self.location[1]
            rotation = self.policy[x][y][0]

            goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
            if x in goal_bounds and y in goal_bounds:
                print self.time_steps
                break 

            action = self.policy[x][y][1]
           
            if count == 0:
                init_rotation = rotation

                if init_rotation == -90:
                    self.heading = self.dir_sensors[self.heading][0]
                elif init_rotation == 90:
                    self.heading = self.dir_sensors[self.heading][2]
                elif init_rotation == 0:
                    pass

            if 0 < count < 3:
                if rotation == -90:
                    break
                elif rotation == 90:
                    break
                elif rotation == 0:
                    pass

            if count == 3:
                break        

            self.location[0] = x + self.dir_move[action][0]
            self.location[1] = y + self.dir_move[action][1]
            
            count += 1

        self.time_steps += 1    
        print init_rotation, count     
        return init_rotation, count

    
    #return the heuristic value of the cell, which is  the manhattan distance from the cell to the center of the goal        
    def heuristic(self, cell):
        return abs(cell[0] - self.maze_dim/2) + abs(cell[1] - self.maze_dim/2)
    

    def A_star_search(self):
        init = [0, 0]
        cost = 1
        heading = 'u'
        goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]

        closed = [[0 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
        closed[init[0]][init[1]] = 1

        #the action taken that leads to the current cell 
        action = [[' ' for row in range(self.maze_dim)] for col in range(self.maze_dim)]
        #the rotations taken that leads to the current cell
        rotations = [[-1 for row in range(self.maze_dim)] for col in range(self.maze_dim)]
        #the rotation and action that the cell's gonna take
        policy = [[(-1, ' ') for row in range(self.maze_dim)] for col in range(self.maze_dim)]

        x = init[0]
        y = init[1]
        g = 0
        f = g + self.heuristic(init)
        frontier = [[f, x, y, heading]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand

        while not found and not resign:
            if not frontier:
                resign = True
                return 'fail'

            else:
                #print "have frontier: {}".format(frontier)    
                frontier.sort()
                frontier.reverse()
                choice = frontier.pop()
                x = choice[1]
                y = choice[2]
                f = choice[0]
                heading = choice[3]
                g = f - self.heuristic([x, y])

                #if goal found, add the policy from the goal back to the root
                if x in goal_bounds and y in goal_bounds:
                    while x != init[0] or y != init[1]:
                        x_p = x - self.dir_move[action[x][y]][0]
                        y_p = y - self.dir_move[action[x][y]][1]

                        policy[x_p][y_p] = (rotations[x][y], action[x][y])

                        x = x_p
                        y = y_p

                    found = True         
 
                wall = self.map[x][y]
                #expand the current node, add the resulting nodes to the frontier if there are no walls in the directions and the nodes are not closed
                for i in [1, 2, 4, 8]:
                    if wall & i != 0:
                        step = self.dir_move[self.action[str(i)]]
                        x2 = x + step[0]
                        y2 = y + step[1]
                        wall2 = self.map[x2][y2]
                        if wall2 != -1 and closed[x2][y2] == 0:
                            g2 = g + cost
                            f2 = g2 + self.heuristic([x2, y2])

                            action[x2][y2] = self.action[str(i)]

                            rotation_directions = self.dir_sensors[heading]
                    
                            rotation = self.dir_angle[str(rotation_directions.index(self.action[str(i)]))]
                            rotations[x2][y2] = rotation

                            heading2 = heading
                            if rotation == -90:
                                heading2 = self.dir_sensors[heading][0]
                            elif rotation == 90:
                                heading2 = self.dir_sensors[heading][2]
                            elif rotation == 0:
                                pass

                            frontier.append([f2, x2, y2, heading2])    
                            
                            closed[x2][y2] = 1


        return policy













