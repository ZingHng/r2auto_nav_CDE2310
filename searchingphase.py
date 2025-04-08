import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import matplotlib.pyplot as plt
from PIL import Image
import math
import cmath
import scipy.stats
import time



# constants
occ_bins = [-1, 0, 50, 100] # -1: unknown cell, 0-50: empty cells, 51-100: wall cells
stop_distance_from_obstacle = 0.25 # distance to obstacle to activate obstacleavoidance, 0.18 good for turtlebot only
#stop_distance_from_obstacle_behind = 3 # distance from obstacle behind to activate obstacleavoidance. this is to account for launcher behind turtlebot
rotatechange = 0.3 # speed of rotation
radius = 20 # los limit of heat sensor
cone_angle = 15 # los limit of heat sensor
minimum_cost = 150 # minimum cost required for circle on costmap to be selected for next decisionpoint so that some obscure corner would not be selected


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians



# Check if there is a clear line of sight between two coordinates using Bresenham's line algorithm
def is_clear_line_of_sight(odata, y1, x1, y2, x2):
    dy = abs(y2 - y1)
    dx = abs(x2 - x1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        # Check if there is an obstacle at the current point
        if odata[y1][x1] == 0:  # Obstacle found
            return False
        
        if (x1, y1) == (x2, y2):  # Reached the target
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return True

# Increment all grids within the given angle range (input_angle ± 10 degrees)
def increment_los_in_angle_range(odata, y, x, input_angle, radius, cone_angle):
    # Define the cone's angle limits
    min_angle = input_angle - cone_angle
    max_angle = input_angle + cone_angle

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            # Check if the point is within the radius
            if dy**2 + dx**2 <= radius**2:
                ny, nx = y + dy, x + dx
                if 0 <= ny < len(odata) and 0 <= nx < len(odata[0]):
                    if odata[ny][nx] != 0:  # Only process empty spaces (>0)
                        # Calculate the angle of the line between (y, x) and (ny, nx)
                        angle = math.degrees(math.atan2(ny - y, nx - x))

                        # Normalize the angle to be within -180 to 180
                        if angle < -180:
                            angle += 360
                        elif angle > 180:
                            angle -= 360

                        # Check if the angle is within the cone (input_angle ± 10 degrees)
                        if min_angle <= angle <= max_angle:
                            if is_clear_line_of_sight(odata, y, x, ny, nx):
                                odata[ny][nx] += 1  # Set the value to 2 to mark it as "seen"
                                



def transform_costmap(costmap, shift_x=0, shift_y=0):
    """
    Transform the given 2D costmap by shifting in the x and y directions.
    
    Args:
        costmap (np.array): Original 2D costmap.
        shift_x (int): Number of units to shift in the x direction (positive for right).
        shift_y (int): Number of units to shift in the y direction (positive for down).
    
    Returns:
        np.array: Transformed costmap.
    """
    # Get the dimensions of the original costmap
    original_height, original_width = costmap.shape
    
    # Create a new costmap with padding
    new_height = original_height + shift_y
    new_width = original_width + shift_x
    transformed_costmap = np.zeros((new_height, new_width), dtype=int)
    
    # Copy the original costmap into the new costmap at the correct shifted position
    for y in range(original_height):
        for x in range(original_width):
            # Calculate the new position after applying the shifts
            new_y = y + shift_y
            new_x = x + shift_x
            
            # Ensure the original position falls within the bounds of the new costmap
            if 0 <= new_y < new_height and 0 <= new_x < new_width:
                transformed_costmap[new_y, new_x] = costmap[y, x]
    
    return transformed_costmap

def transform_costmap_back(costmap, shift_x=0, shift_y=0):
    """
    Transform the given 2D costmap by shifting in the x and y directions,
    adding 0s at the back (bottom for y and right for x).
    
    Args:
        costmap (np.array): Original 2D costmap.
        shift_x (int): Number of units to shift in the x direction (positive for right).
        shift_y (int): Number of units to shift in the y direction (positive for down).
    
    Returns:
        np.array: Transformed costmap.
    """
    # Get the dimensions of the original costmap
    original_height, original_width = costmap.shape
    
    # Create a new costmap with padding added at the back
    new_height = original_height + shift_y
    new_width = original_width + shift_x
    transformed_costmap = np.zeros((new_height, new_width), dtype=int)
    
    # Copy the original costmap into the new costmap at the correct shifted position
    for y in range(original_height):
        for x in range(original_width):
            # Calculate the new position after applying the shifts
            new_y = y
            new_x = x
            
            # Ensure the original position falls within the bounds of the new costmap
            if 0 <= new_y < new_height and 0 <= new_x < new_width:
                transformed_costmap[new_y, new_x] = costmap[y, x]
    
    return transformed_costmap


class Searchingphase(Node):
    def __init__(self):
        super().__init__('searchingphase')
        
        # create subscription for map
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        # used for finding robot's position
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.grid_x = None # x position of robot on map from /map topic
        self.grid_y = None # y position of robot on map from /map topic
        self.odata = np.array([]) # store the 2D aaray of map from /map topic
        self.map_origin = None # store map origin to facilitate transformation of coordinates
        # store the 2D array of map of spaces that have been seen by heat sensor
        self.costmap = np.array([
        [0, 0],
        [0, 0]
        ]) 
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            50)
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.old_yaw = 0
        self.yaw_changed = False

        '''
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.obstacle_angle = 0
        self.obstacle = False
        '''

        # create pathfinderactive subscription, send new decisionpoint when pathfinderactive is false
        self.pathfinderactive_subscription = self.create_subscription(
            Bool,
            'pathfinderactive',
            self.pathfinderactive_callback,
            10)
        self.pathfinderactive_subscription
        self.makingdecision = False # whether robot is at decision point 

        # create mappingphaseactive subscription to check if can proceed with mappingphase
        self.mappingphaseactive_subscription = self.create_subscription(
            Bool,
            'mappingphaseactive',
            self.mappingphaseactive_callback,
            10)
        self.mappingphaseactive_subscription
        self.mappingphaseactive = True # DEFAULT IS TRUE

        # create subscription to survivorzonesequence
        self.szs_subscription = self.create_subscription(
            Bool,
            'survivorzonesequenceactive',
            self.szs_callback,
            10)
        self.szsactive = False

        # create cmd_vel publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        
        # create decisionpoint publisher to send pathfinder new decision points
        self.dp_publisher_ = self.create_publisher(Point, 'decisionpoint', 10) # publishes new decision point for pathfinder node
        self.decisionpoint = None # (y, x) coordinates of next decision point       

        self.survivorsfound = 0

    def occ_callback(self, msg):
        occdata = np.array(msg.data) # create numpy array
        # compute histogram to identify bins with -1, values between 0 and below 50, 
        # and values between 50 and 100. The binned_statistic function will also
        # return the bin numbers so we can use that easily to create the image 
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins) #tbh idk why there are 3 variables so imma just leave it

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        self.odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        # change odata such that it only cares about empty spaces
        self.odata[self.odata == 1] = 0
        self.odata[self.odata == 2] = 1
        self.odata[self.odata == 3] = 0

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
        map_res = msg.info.resolution # get map resolution
        if (self.grid_x is not None) and (self.grid_y is not None):
            old_grid_x = self.grid_x # save previous robot location to see if need to update costmap
            old_grid_y = self.grid_y  
        
        self.grid_x = round((cur_pos.x - self.map_origin.x) / map_res) # x position of robot on map from /map topic
        self.grid_y = round(((cur_pos.y - self.map_origin.y) / map_res)) # y posiiton of robot on map from /map topic  

        # update costmap if map expands/origin shifts
        if (self.map_origin is not None) and (old_map_origin is not None) and (not old_map_origin == self.map_origin):
            shift_x = round((old_map_origin.x - self.map_origin.x) / map_res)
            shift_y = round((old_map_origin.y - self.map_origin.y) / map_res)
            self.costmap = transform_costmap(self.costmap, shift_x, shift_y)
            print('Transformed costmap 1')

        if (len(self.odata[0]) != len(self.costmap[0])) or (len(self.odata) != len(self.costmap)):
            shift_x = len(self.odata[0]) - len(self.costmap[0])
            shift_y = len(self.odata) - len(self.costmap)
            self.costmap = transform_costmap_back(self.costmap, shift_x, shift_y)
            print('Transformed costmap 2')
        
        # Update the costmap to include new empty grids
        if (len(self.odata[0]) == len(self.costmap[0])) and (len(self.odata) == len(self.costmap)):
            self.costmap[(self.odata == 1) & (self.costmap == 0)] = 1

        # update cost map if robot has moved
        if (old_map_origin is None) or ((not old_grid_x == self.grid_x) or (not old_grid_y == self.grid_y)) or (not old_map_origin == self.map_origin) or self.yaw_changed:
            increment_los_in_angle_range(self.costmap, self.grid_y, self.grid_x, math.degrees(self.yaw), radius, cone_angle)
            self.yaw_changed = False

        img = Image.fromarray(self.costmap.astype(np.uint8))
        # show the image using gradient map
        plt.imshow(img, cmap='Oranges', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        # updates robot roll, pitch and yaw
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        if abs(self.yaw - self.old_yaw) > math.radians(cone_angle):
            self.yaw_changed = True
            self.old_yaw = self.yaw
            

    '''
    # mainly to check for obstacles and initiate obstacle avoidance    
    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges) # create numpy array of laser scans
        self.laser_range[self.laser_range==0] = np.nan # replace 0's with nan
        lr2i = np.nanargmin(self.laser_range)        
        # if closest point is less than stop distance, initiate obstacle avoidance sequence and set obstacle to true to break while loops in purepursuit/rotatebot
        if self.laser_range[lr2i] < stop_distance_from_obstacle:
            print('OBSTACLE DETECTED')
            # convert lr2i to real world angle
            rot_angle = lr2i + math.degrees(self.yaw)
            rot_angle = rot_angle % 360
            # If the angle is greater than 180, subtract 360 to bring it to the range [-180, 180)
            if rot_angle > 180:
                rot_angle -= 360
            print(f"Obstacle Angle: {rot_angle}")
            self.obstacle_angle = rot_angle
            self.obstacle = True
    '''
    
    def mappingphaseactive_callback(self, msg):
        self.mappingphaseactive = msg.data
        if self.mappingphaseactive is False:
            print('MAPPINGPHASEACTIVE COMPLETE')
            self.decisionpointselect()

    def pathfinderactive_callback(self, msg):
        if self.mappingphaseactive is False:
            print(f'pathfinderactive: {msg.data}')
            self.makingdecision = not (msg.data)
            if self.makingdecision:
                print('DECISION POINT')

    def szs_callback(self, msg):
        print('SURVIVOR ZONE SEQUENCE ACTIVE')
        self.szsactive = msg.data

    def fullrotation(self):
        # rotate the bot 360
        print('fullrotation')
        yaw = self.yaw
        twist = Twist()
        twist.linear.x = 0.0 # set linear speed to zero so the TurtleBot rotates on the spot
        twist.angular.z = rotatechange # set to rotate
        self.publisher_.publish(twist) # start rotation
        time.sleep(2)
        rclpy.spin_once(self)
        while True:
            rclpy.spin_once(self)
            if 0 < math.degrees(abs(self.yaw-yaw)) < 10:
             break
            if self.szsactive:
                break
        twist.linear.x = 0.0 
        twist.angular.z = 0.0
        self.publisher_.publish(twist) # stop rotation
        print('finishedrotation')                          


    def decisionpointselect(self):
        def is_valid_coordinate(y, x, costmap):
            """Check if the coordinates are within the bounds of the costmap."""
            return 0 <= y < costmap.shape[0] and 0 <= x < costmap.shape[1]

        def get_circle_indices(y_center, x_center, radius, costmap):
            """Get the indices of the points within a circle of a given radius."""
            indices = []
            for y in range(y_center - radius, y_center + radius + 1):
                for x in range(x_center - radius, x_center + radius + 1):
                    # Check if the point is within the circle
                    if (y - y_center) ** 2 + (x - x_center) ** 2 <= radius ** 2:
                        if is_valid_coordinate(y, x, costmap):
                            indices.append((y, x))
            return indices

        def is_visible(y1, x1, y2, x2, costmap):
            """Determine if (y2, x2) is visible from (y1, x1), considering obstacles."""
            # Use Bresenham's line algorithm to check for visibility between two points
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            sx = 1 if x1 < x2 else -1
            sy = 1 if y1 < y2 else -1
            err = dx - dy
            
            while True:
                if not is_valid_coordinate(y1, x1, costmap):
                    return False
                if costmap[y1, x1] == 0:  # obstacle found
                    return False
                if (y1, x1) == (y2, x2):
                    break
                e2 = err * 2
                if e2 > -dy:
                    err -= dy
                    x1 += sx
                if e2 < dx:
                    err += dx
                    y1 += sy
            return True

        def get_cost_of_circle(y_center, x_center, radius, costmap):
            """Calculate the total cost of a circle around (y_center, x_center), accounting for visibility."""
            indices = get_circle_indices(y_center, x_center, radius, costmap)
            total_cost = 0
            for (y, x) in indices:
                if costmap[y, x] == 0:  # Skip obstacles entirely
                    continue
                
                # Check if (y, x) is visible from the center
                if is_visible(y_center, x_center, y, x, costmap):
                    total_cost += costmap[y, x]
                
            return total_cost

        def find_best_location(costmap, radius):
            """Find the best location to start a 360-degree sweep."""
            best_location = None
            best_cost = float('inf')
            
            for y in range(costmap.shape[0]):
                for x in range(costmap.shape[1]):
                    # Skip checking if the point is an obstacle
                    if costmap[y, x] == 0:
                        continue

                    # Calculate the cost of the circle around this point
                    total_cost = get_cost_of_circle(y, x, radius, costmap)

                    # If this point has a lower cost, update the best location
                    if total_cost < best_cost and minimum_cost < total_cost:
                        best_cost = total_cost
                        best_location = (y, x)
                        #print(best_cost)
                        #print(best_location)

            return best_location


        best_location = find_best_location(self.costmap, radius)
        self.decisionpoint = best_location
        if (self.decisionpoint is not None):
            self.makingdecision = False
            print(f'Decisionpoint: {self.decisionpoint}')
            point = Point()
            point.y = float(round(self.decisionpoint[0]))
            point.x = float(round(self.decisionpoint[1]))
            self.dp_publisher_.publish(point) # publish new dp for pathplanner



def main(args=None):
    rclpy.init(args=args)
    searchingphase = Searchingphase()

    # create matplotlib figure
    plt.ion()
    plt.show()    

    while True:
        rclpy.spin_once(searchingphase)
        if (searchingphase.makingdecision):
            searchingphase.fullrotation()
            if not searchingphase.szsactive:
                searchingphase.decisionpointselect()            
        '''
        if (searchingphase.survivorsfound == 2):
            # ramp sequence
        '''
        # freeze pathfinder while szsactive
        while searchingphase.szsactive:
            rclpy.spin_once(searchingphase)
            print('stuck in szs')



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frontier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()