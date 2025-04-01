# this node subscribes to /map topic and decides the subsequent decision point whenever robot reaches a decision point
# this node publishes the subsequent decision point for the pathfinder node to create path and execute pure pursuit to the subsequent decision point
# this node will also publish when mapping phase is done so that the searching phase node can become active

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
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point

# constants
occ_bins = [-1, 0, 50, 100] # -1: unknown cell, 0-50: empty cells, 51-100: wall cells
minimum_frontier_length = 8
do_not_cross_line = 2

class Frontier(Node):

    def __init__(self):
        super().__init__('frontier')
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
        self.decisionpoint = None # (y, x) coordinates of next decision point 
        self.pathfinderactive_subscription = self.create_subscription(
            Bool,
            'pathfinderactive',
            self.pathfinderactive_callback,
            10)
        self.pathfinderactive_subscription
        self.makingdecision = True # whether robot is at decision point 
        self.mappingphase = True

        self.dp_publisher_ = self.create_publisher(Point, 'decisionpoint', 10) # publishes new decision point for pathfinder node
        self.mappingphase_publisher_ = self.create_publisher(Bool, 'mappingphaseactive', 10) # publishes that mapping phase is active, stops searching phase from commencing
        self.map_origin = None

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

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return    
        cur_pos = trans.transform.translation # real world coordinates of robot relative to robot start point

        if (self.map_origin is not None):
            old_map_origin = self.map_origin # save previous map origin to update decision point relative to new map later
        self.map_origin = msg.info.origin.position # real world coordinates of the origin of map from /map topic relative to robot start point
        map_res = msg.info.resolution # get map resolution
        self.grid_x = round((cur_pos.x - self.map_origin.x) / map_res) # x position of robot on map from /map topic
        self.grid_y = round(((cur_pos.y - self.map_origin.y) / map_res)) # y posiiton of robot on map from /map topic
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (self.grid_y, self.grid_x))

        # update dp based on new map
        if (self.decisionpoint is not None) and (self.map_origin is not None) and (not old_map_origin == self.map_origin):
            self.decisionpoint = (round(self.decisionpoint[0] + (old_map_origin.y - self.map_origin.y) / map_res), round(self.decisionpoint[1] + (old_map_origin.x - self.map_origin.x) / map_res))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        self.odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))

        # draw do not cross line as wall so that robot does not see ramp as frontier
        if msg.info.height > round(do_not_cross_line / map_res):
            start_row = round(do_not_cross_line / map_res)
            for row in range(start_row, msg.info.height):
                self.odata[row] = [3] * msg.info.width

        self.odata[self.grid_y][self.grid_x] = 0 # set current robot location to 0 to see on the matplotlib
        if (self.decisionpoint is not None):
            self.odata[int(self.decisionpoint[0]), int(self.decisionpoint[1])] = 0 # set decision point location to 0 to see on the matplotlib
        img = Image.fromarray(self.odata) # create image from 2D array using PIL

        # show the image using grayscale map
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

        
            
        

        # send this code to pathfinder instead
        '''
        # check if robot has reached decision point
        if (self.decisionpoint is not None):
            y_dist_to_dp, x_dist_to_dp = np.abs(self.grid_y - self.decisionpoint[0]), np.abs(self.grid_x - self.decisionpoint[1])
            if (y_dist_to_dp < stop_distance_from_dp) and (x_dist_to_dp < stop_distance_from_dp):
                print('DECISION POINT')
                self.makingdecision = True
        '''

        print(f"self.grid_y: {self.grid_y}")
        print(f"self.grid_x: {self.grid_x}")
        print(f"self.decisionpoint: {self.decisionpoint}")

    def frontiersearch(self):        
        # 8 directions of neighbours
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        # searching for all frontier cells
        self.frontiercells = []
        for grid_y in range(len(self.odata)):  # Iterate through rows
            for grid_x in range(len(self.odata[grid_y])):  # Iterate through columns
                if self.odata[grid_y][grid_x] == 1:  # Check if the value is 1
                    # Check all 8 neighbors
                    for dy, dx in directions:
                        ny, nx = grid_y + dy, grid_x + dx
                        # Check if the neighbor is within bounds and contains 2
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
        # check if it is last decision point
        if (self.decisionpoint is not None) and (closest_coordinate is None):
            self.mappingphase = False
            msg = Bool()
            msg.data = self.mappingphase
            self.mappingphase_publisher_.publish(msg)
        
        self.decisionpoint = closest_coordinate
        
        if (self.decisionpoint is not None):
            self.makingdecision = False
            point = Point()
            point.y = float(round(self.decisionpoint[0]))
            point.x = float(round(self.decisionpoint[1]))
            self.dp_publisher_.publish(point) # publish new dp for pathplanner

def main(args=None):
    rclpy.init(args=args)
    frontier = Frontier()

    # create matplotlib figure
    plt.ion()
    plt.show()    

    while True:
        rclpy.spin_once(frontier)
        if (frontier.makingdecision) and (frontier.mappingphase):
            frontier.frontiersearch()
            frontier.frontierselect()
        if (not frontier.mappingphase):
            break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frontier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()