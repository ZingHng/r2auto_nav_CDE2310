# this node subscribes to /map topic and decides the subsequent decision point whenever robot reaches a decision point
# this node publishes the subsequent decision point to /decisionpoint topic for the pathfinder node to create path and execute pure pursuit to the subsequent decision point
# this node will also publish if mapping phase is active to /mappingphaseactive so that the searching phase node will know when to become active

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import scipy.stats
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import time

# constants
occ_bins = [-1, 0, 50, 100] # -1: unknown cell, 0-50: empty cells, 51-100: wall cells
minimum_frontier_length = 15
do_not_cross_line = -3.4 # about 3/-3? depends on direction
dp_after_dncline = (0.25, 3.2) # (y, x), but only x is used

class Frontier(Node):

    def __init__(self):
        super().__init__('frontier')
        
        # create subscription for map
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        # used for finding robots position
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.grid_x = None # x position of robot on map from /map topic
        self.grid_y = None # y position of robot on map from /map topic
        self.odata = np.array([]) # store the 2D aaray of map from /map topic
        self.frontiercells = np.array([]) # store the cells that may be part of frontiers
        self.frontierlist = np.array([]) # store groups of cells that form frontiers
        self.map_origin = None # store map origin to facilitate transformation of coordinates
        self.map_res = 0
        self.oldest_height = None
        self.oldest_map_origin = None
        self.dp_after_dncline_y = None
        
        # create pathfinderactive subscription, send new decisionpoint when pathfinderactive is false
        self.pathfinderactive_subscription = self.create_subscription(
            Bool,
            'pathfinderactive',
            self.pathfinderactive_callback,
            10)
        self.pathfinderactive_subscription
        self.makingdecision = True # whether robot is at decision point 

        # create subscription to survivorzonesequence
        self.szs_subscription = self.create_subscription(
            Bool,
            'survivorzonesequenceactive',
            self.szs_callback,
            10)
        self.szsactive = False

        # create mappingphaseactive subscription to check if can proceed with mappingphase
        self.mappingphaseactive_subscription = self.create_subscription(
            Bool,
            'mappingphaseactive',
            self.mappingphaseactive_callback,
            10)
        self.mappingphaseactive_subscription
        self.mappingphaseactive = False # DEFAULT IS FALSE 

        # create decisionpoint publisher to send pathfinder new decision points
        self.dp_publisher_ = self.create_publisher(Point, 'decisionpoint', 10) # publishes new decision point for pathfinder node
        self.decisionpoint = None # (y, x) coordinates of next decision point 

        '''
        # create mappingphaseactive publisher, stops searching phase from commencing
        self.mappingphaseactive_publisher_ = self.create_publisher(Bool, 'mappingphaseactive', 10) 
        self.mappingphaseactive = True
        '''

        # create targetlock publisher to tell pathfinder to turn on targetlock for final dp
        self.targetlock_publisher_ = self.create_publisher(Bool, 'targetlock', 10)

        # create rampsequence publisher to tell rampsequence to run
        self.rampsequence_publisher_ = self.create_publisher(Bool, 'rampsequence', 10)
        self.rampsequence = False

    def pathfinderactive_callback(self, msg):
        self.makingdecision = not (msg.data)
        print('DECISION POINT')

    # occ_callback gets called each time there is an update to /map topic
    def occ_callback(self, msg):
        # create numpy array
        occdata = np.array(msg.data)
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins) #tbh idk why there are 3 variables so imma just leave it

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        self.odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return    
        cur_pos = trans.transform.translation # real world coordinates of robot relative to robot start point
        
        old_map_origin = None
        if (self.map_origin is not None):
            old_map_origin = self.map_origin # save previous map origin to update decision point relative to new map later
        self.map_origin = msg.info.origin.position # real world coordinates of the origin of map from /map topic relative to robot start point
        if self.oldest_map_origin is None:
            self.oldest_map_origin = self.map_origin
        if self.oldest_height is None:
            self.oldest_height = msg.info.height
        map_res = msg.info.resolution # get map resolution
        self.map_res = map_res
        if (cur_pos is not None):
            self.grid_x = round((cur_pos.x - self.map_origin.x) / map_res) # x position of robot on map from /map topic
            self.grid_y = round(((cur_pos.y - self.map_origin.y) / map_res)) # y posiiton of robot on map from /map topic

        # update dp based on new map
        if (self.decisionpoint is not None) and (self.map_origin is not None) and (not old_map_origin == self.map_origin):
            self.decisionpoint = (round(self.decisionpoint[0] + (old_map_origin.y - self.map_origin.y) / map_res), round(self.decisionpoint[1] + (old_map_origin.x - self.map_origin.x) / map_res))
            print(f"transformed self.decisionpoint: {self.decisionpoint}")

        # draw do not cross line as wall so that robot does not see ramp as frontier
        if msg.info.height > round(abs(do_not_cross_line / map_res)):
            if do_not_cross_line < 0:
                start_row = (msg.info.height - self.oldest_height) - round((self.oldest_map_origin.y - self.map_origin.y)/map_res) + round(abs(do_not_cross_line / map_res)) 
                self.dp_after_dncline_y = msg.info.height - start_row - 25 # 25 grids before line
                #print(start_row)
                for row in range(0, msg.info.height - start_row):
                    self.odata[row] = [3] * msg.info.width
            if do_not_cross_line > 0:
                start_row = round((self.oldest_map_origin.y - self.map_origin.y)/map_res) + round(abs(do_not_cross_line / map_res))
                self.dp_after_dncline_y = start_row + 25 # 25 grids before line
                for row in range(start_row, msg.info.height):
                    self.odata[row] = [3] * msg.info.width

        self.odata[self.grid_y][self.grid_x] = 0 # set current robot location to 0 to see on the matplotlib
        if (self.decisionpoint is not None):
            self.odata[int(self.decisionpoint[0]), int(self.decisionpoint[1])] = 0 # set decision point location to 0 to see on the matplotlib
        img = Image.fromarray(self.odata) # create image from 2D array using PIL

        # show the image using gradient map
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

        #print(f"self.grid_y: {self.grid_y}")
        #print(f"self.grid_x: {self.grid_x}")
        #print(f"self.decisionpoint: {self.decisionpoint}")

    def szs_callback(self, msg):
        self.szsactive = msg.data
        if self.szsactive:
            print('SURVIVOR ZONE SEQUENCE ACTIVE')
        else:
            print('SURVIVOR ZONE SEQUENCE COMPLETE')

    def mappingphaseactive_callback(self, msg):
        self.mappingphaseactive = msg.data
        if self.mappingphaseactive is False:
            print('MAPPINGPHASEACTIVE START')
            self.decisionpointselect()

    def frontiersearch(self):        
        # 8 directions of neighbours
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        # searching for all frontier cells
        self.frontiercells = []
        for grid_y in range(len(self.odata)):  # Iterate through rows
            for grid_x in range(len(self.odata[grid_y])):  # Iterate through columns
                if self.odata[grid_y][grid_x] == 1:  # Check if the value is 1 which is unknown cell
                    # Check all 8 neighbors
                    for dy, dx in directions:
                        ny, nx = grid_y + dy, grid_x + dx
                        # Check if the neighbor is within bounds and contains 2 which is empty cell
                        if 0 <= ny < len(self.odata) and 0 <= nx < len(self.odata[grid_y]) and self.odata[ny][nx] == 2:
                            self.frontiercells.append((grid_y, grid_x))
                            break  # Once we find a neighbor with value 2, stop checking further neighbors
        # to see frontier cells
        np.savetxt('frontiercells.txt', self.frontiercells)
    
        # grouping frontier cells which form a frontier using dfs
        def dfs(coord, visited, group):
            # Mark the current coordinate as visited and add it to the group
            visited.add(coord)
            group.append(coord)
            # Explore all 8 neighboring cells
            for dy, dx in directions:
                ny, nx = coord[0] + dy, coord[1] + dx
                neighbor = (ny, nx)
                # If the neighbor is in coordinates and not visited, visit it
                if neighbor in self.frontiercells and neighbor not in visited:
                    dfs(neighbor, visited, group)
        self.frontierlist = [] # store frontiers
        visited = set()
        for cell in self.frontiercells:
            if cell not in visited:
                # Start a new group if the coordinate has not been visited
                group = []
                dfs(cell, visited, group)
                if (len(group)>minimum_frontier_length):
                    self.frontierlist.append(group)
        
        # filter out stray frontiers from broken walls
        # Helper function to check Euclidean distance between two points
        def is_within_range(p1, p2, max_dist=5):
            return math.hypot(p1[0] - p2[0], p1[1] - p2[1]) <= max_dist
        # Collect endpoints of all frontiers
        endpoints = []
        for frontier in self.frontierlist:
            if len(frontier) >= 2:
                endpoints.append((frontier[0], frontier[-1]))  # start and end points
            else:
                endpoints.append((frontier[0], frontier[0]))  # same point repeated
        # Mark which frontiers should be removed
        remove_indices = set()
        for i in range(len(endpoints)):
            for j in range(i + 1, len(endpoints)):
                ends_i = endpoints[i]
                ends_j = endpoints[j]
                # Check all combinations of endpoints between two frontiers
                for p1 in ends_i:
                    for p2 in ends_j:
                        if is_within_range(p1, p2, max_dist=minimum_frontier_length):
                            remove_indices.add(i)
                            remove_indices.add(j)
        # Remove frontiers that are too close at the ends
        self.frontierlist = [f for idx, f in enumerate(self.frontierlist) if idx not in remove_indices]
        
        # to see frontier list
        frontierlist_as_strings = [str(row) for row in self.frontierlist]
        np.savetxt('frontierlist.txt', frontierlist_as_strings, fmt='%s')

    def frontierselect(self):
        # check if decision point was previously None
        # Function to calculate the average coordinate for each group
        def average_coordinates(frontiercells):
            # Convert each tuple in the group to numpy arrays for easy manipulation
            y_values = [cell[0] for cell in frontiercells]
            x_values = [cell[1] for cell in frontiercells]
            
            # Calculate the average of x and y values
            avg_y = np.mean(y_values)
            avg_x = np.mean(x_values)
            
            return (avg_y, avg_x)
        # Find the average coordinate for each frontier and store in a list
        averages = [average_coordinates(frontier) for frontier in self.frontierlist]
        # the average coordinate for each frontier is a potential new decision point
        np.savetxt('decisionpoints.txt', averages)

        # Function to calculate Euclidean distance between two points (x1, y1) and (x2, y2)
        def euclidean_distance(coord1, coord2):
            return np.sqrt((coord2[0] - coord1[0])**2 + (coord2[1] - coord1[1])**2)
        robotposition = (self.grid_y, self.grid_x) 
        min_distance = float('inf') # Find the coordinate in 'averages' with the lowest distance to the input coordinate
        closest_coordinate = None
        for coord in averages:
            dist = euclidean_distance(robotposition, coord)
            if dist < min_distance:
                min_distance = dist
                closest_coordinate = coord
        # if there are no moore decisionpoints, then mappingphase is complete
        if (closest_coordinate is None):
            self.mappingphaseactive = False
            print('MAPPINGPHASE COMPLETED')            
        
        self.decisionpoint = closest_coordinate
        
        if (self.decisionpoint is not None):
            self.makingdecision = False
            point = Point()
            point.y = float(round(self.decisionpoint[0]))
            point.x = float(round(self.decisionpoint[1]))
            self.dp_publisher_.publish(point) # publish new dp for pathplanner
            print(f"self.decisionpoint: {self.decisionpoint}")
        
    def pastdncline(self):
        while True:
            rclpy.spin_once(self) 
            if self.map_res != 0:
                break
        y, x = dp_after_dncline
        self.decisionpoint = (self.dp_after_dncline_y, round(x / self.map_res))
        self.makingdecision = False
        print(f'Decisionpoint: {self.decisionpoint}')
        msg = Bool()
        msg.data = True
        self.targetlock_publisher_.publish(msg)
        time.sleep(1)
        point = Point()
        point.y = float(round(self.decisionpoint[0]))
        point.x = float(round(self.decisionpoint[1]))
        self.dp_publisher_.publish(point) # publish new dp for pathplanner
        print(f"MOVING TO FINAL WAYPOINT {self.decisionpoint}")

def main(args=None):
    rclpy.init(args=args)
    frontier = Frontier()

    # create matplotlib figure
    plt.ion()
    plt.show()    

    mappingphasestarted = False
    while True:
        rclpy.spin_once(frontier)
        while frontier.mappingphaseactive:
            mappingphasestarted = True
            rclpy.spin_once(frontier)
            if (frontier.makingdecision) and (frontier.mappingphaseactive) and (frontier.grid_x is not None) and (frontier.grid_y is not None):
                    frontier.frontiersearch()
                    frontier.frontierselect()
            
            '''
            # freeze pathfinder while szsactive
            while frontier.szsactive:
                rclpy.spin_once(frontier)
                print('stuck in szs')
            '''

            if not frontier.mappingphaseactive:
                break
        if mappingphasestarted:
            break


    frontier.pastdncline()
    while frontier.makingdecision is not True:
        rclpy.spin_once(frontier)
    frontier.rampsequence = True
    msg = Bool()
    msg.data = frontier.rampsequence
    frontier.rampsequence_publisher_.publish(msg)
    time.sleep(5)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frontier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()