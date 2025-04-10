# this node subscribes to /descisionpoint, /map and /odom topic and tries to navigate to decisionpoint
# it creates an optimal path by overlaying an A* cost map on a wall proximity cost map and finding the path with the lowest total cost
# waypoints are created based on the optimal path and the bot tries to move from waypoint to waypoint
# this node also subscribes to /scan topic and initiates obstacle avoidance if lidar detects an obstacle
# targetlock True means the algorithm will keep forcing it's way to decisionpoint even after it detects and avoids obstacles
# targetlock False means the algorithm will move on to new decisionpoint after it detects and avoids obstacles
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
import heapq
import time



# constants
occ_bins = [-1, 0, 50, 100] # -1: unknown cell, 0-50: empty cells, 51-100: wall cells
stop_distance_from_dp = 8 # distance from decision point to stop pathfinder
stop_distance_from_wp = 4 # distance from waypoint to change to next waypoint
stop_distance_from_obstacle = 0.18 # distance to obstacle to activate obstacleavoidance, 0.18 good for turtlebot only
stop_distance_from_obstacle_behind = 0.3 # distance from obstacle behind to activate obstacleavoidance. this is to account for launcher behind turtlebot
waypoint_gap = 5 # number of grids between pure pursuit waypoints
rotatechange = 0.3 # speed of rotation
speedchange = 0.12 # speed of linear movement
targetlock = False # whether the algorithm forces it's way to a certain decisionpoint or moves to next decisionpoint in the case of obstacles



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



# class to create optimal path given a map, start point and end point
class AStar:
    def __init__(self, grid, start, end):
        self.grid = grid
        self.start = start
        self.end = end
        self.rows = len(grid)
        self.cols = len(grid[0])        
        self.astar_costmap = [[0] * self.cols for _ in range(self.rows)]
        self.wallprox_costmap = [[0] * self.cols for _ in range(self.rows)]
        self.overall_costmap = [[0] * self.cols for _ in range(self.rows)]
        self.build_astar_costmap()
        self.build_wallprox_costmap()
        for grid_y in range(self.rows):  # Iterate through rows
            for grid_x in range(self.cols): # Iterate through columns
                self.overall_costmap[grid_y][grid_x] = self.astar_costmap[grid_y][grid_x] + self.wallprox_costmap[grid_y][grid_x]
        self.path, self.total_cost = self.dijkstra(self.overall_costmap, self.start, self.end)
        self.print()
        print(f"New Path: {self.path}")
        print(f"Total Cost: {self.total_cost}")
    
    # build f cost map
    def build_astar_costmap(self):
        for grid_y in range(self.rows):  # Iterate through rows
            for grid_x in range(self.cols):  # Iterate through columns
                x_diff = np.abs(grid_x - self.start[1])
                y_diff = np.abs(grid_y - self.start[0])
                g_cost = np.sqrt(x_diff**2 + y_diff**2) # calculate g cost of each grid
                x_diff = np.abs(grid_x - self.end[1])
                y_diff = np.abs(grid_y - self.end[0]) # calculate h cost of each grid
                h_cost = np.sqrt(x_diff**2 + y_diff**2)
                f_cost = g_cost + h_cost
                self.astar_costmap[grid_y][grid_x] = round(f_cost*10)

    # build wall proximity cost map
    def build_wallprox_costmap(self):
        for grid_y in range(self.rows):  # Iterate through rows
            for grid_x in range(self.cols):  # Iterate through columns
                if self.grid[grid_y][grid_x] == 3: # If the current cell is a wall, it is given cost of 10*10
                    self.wallprox_costmap[grid_y][grid_x] = 10**10
                    for dy in range(-8, 9):  # Look within 8 cells distance in y direction
                        for dx in range(-8, 9): # looks withihn 8 cells distance in x direction
                            ny, nx = grid_y + dy, grid_x + dx
                            if 0 <= ny < self.rows and 0 <= nx < self.cols and self.grid[ny][nx] != 3:
                                dist = abs(dy) + abs(dx)
                                if dist == 1:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 10**8) # 1 cell away
                                elif dist == 2:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 100000) # 2 cells away
                                elif dist == 3:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 90000) # 3 cells away
                                elif dist == 4:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 80000) # 4 cells away
                                elif dist == 5:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 65000) # 5 cells away
                                elif dist == 6:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 50000) # 6 cells away
                                elif dist == 7:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 35000) # 7 cells away
                                elif dist == 8:
                                    self.wallprox_costmap[ny][nx] = max(self.wallprox_costmap[ny][nx], 20000) # 8 cells away      

    # chooses the path with the least total cost on a costmap
    def dijkstra(self, grid, start, end):
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        rows, cols = len(grid), len(grid[0])
        pq = [(grid[start[0]][start[1]], start[0], start[1])] # Priority queue with (cost, row, col)
        dist = [[float('inf')] * cols for _ in range(rows)] # Distance matrix initialized to infinity
        dist[start[0]][start[1]] = grid[start[0]][start[1]]        
        prev = [[None] * cols for _ in range(rows)] # Previous matrix to reconstruct the path
        
        while pq:
            current_dist, r, c = heapq.heappop(pq)            
            # If we reach the end point, we can reconstruct the path
            if (r, c) == end:
                path = []
                total_cost = 0  # To keep track of the total cost
                while (r, c) != start:
                    path.append((r, c))
                    total_cost += grid[r][c]  # Add the current cell's value to the total cost
                    r, c = prev[r][c]
                path.append(start)
                total_cost += grid[start[0]][start[1]]  # Add the start cell's value to the total cost
                return path[::-1], total_cost  # Reverse the path and return it along with the total cost
            # Explore neighbors
            for dr, dc in directions:
                nr, nc = r + dr, c + dc                
                if 0 <= nr < rows and 0 <= nc < cols:
                    new_dist = current_dist + grid[nr][nc]
                    if new_dist < dist[nr][nc]:
                        dist[nr][nc] = new_dist
                        prev[nr][nc] = (r, c)
                        heapq.heappush(pq, (new_dist, nr, nc))
        return [], 0  # Return an empty path and 0 if no path is found
    
    # display the frozen figure of path plotted                          
    def print(self):
        pathmap = self.grid
        pathmap[self.start[0]][self.start[1]] = 0
        for (y, x) in self.path:
            pathmap[y, x] = 0
        img = Image.fromarray(pathmap) # create image from 2D array using PIL
        # show the image using gradient map
        plt.imshow(img, cmap='magma', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(1)



class Pathfinder(Node):
    def __init__(self):
        super().__init__('pathfinder')
        
        # create subscription for any incoming new decision point
        self.dp_subscription = self.create_subscription(
            Point,
            'decisionpoint',
            self.dp_callback,
            10)
        self.dp_subscription # prevent unused variable warning
        self.decisionpoint = None 
        
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
        self.reacheddp = False
        self.map_origin = None # store map origin to facilitate transformation of coordinates
        
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
        self.direction = 0 # 1 is forward, 2 is reverse, 0 is neutral

        # create subscription to survivorzonesequence
        self.szs_subscription = self.create_subscription(
            Bool,
            'survivorzonesequenceactive',
            self.szs_callback,
            10)
        self.szsactive = False

        # create pathfinderactive publisher so that mappingphase and searchingphase do not send new decisionpoints
        self.pathfinderactive_publisher_ = self.create_publisher(Bool, 'pathfinderactive', 10) #pathfinderactive publisher
        self.pathfinderactive = False
        self.waypoints = None

        # create cmd_vel publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
                
    # to read decisionpoints
    def dp_callback(self, msg):
        print('NEW DECISION POINT')
        self.decisionpoint = (msg.y, msg.x)
        print(self.decisionpoint)       

    def occ_callback(self, msg):
        occdata = np.array(msg.data) # create numpy array
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

        if (self.map_origin is not None):
            old_map_origin = self.map_origin # save previous map origin to update decision point relative to new map later
        self.map_origin = msg.info.origin.position # real world coordinates of the origin of map from /map topic relative to robot start point
        map_res = msg.info.resolution # get map resolution
        self.grid_x = round((cur_pos.x - self.map_origin.x) / map_res) # x position of robot on map from /map topic
        self.grid_y = round(((cur_pos.y - self.map_origin.y) / map_res)) # y posiiton of robot on map from /map topic
        
        # if decision point is reached, set reacheddp to true, which will lead to killpathfinder in purepursuit
        if (self.decisionpoint is not None):
            y_dist_to_dp = np.abs(self.grid_y - self.decisionpoint[0])
            x_dist_to_dp = np.abs(self.grid_x - self.decisionpoint[1])
            if (y_dist_to_dp < stop_distance_from_dp) and (x_dist_to_dp < stop_distance_from_dp):
                print(f"Reached dp: {y_dist_to_dp}, {x_dist_to_dp}")
                self.reacheddp = True    

        # update decision point if map origin changes
        if (self.decisionpoint is not None) and (self.map_origin is not None) and (not old_map_origin == self.map_origin):
            self.decisionpoint = (round(self.decisionpoint[0] + (old_map_origin.y - self.map_origin.y) / map_res), round(self.decisionpoint[1] + (old_map_origin.x - self.map_origin.x) / map_res))
            print(f"Transformed Decisionpoint: {self.decisionpoint}")

        # update path if map origin changes
        if (self.waypoints is not None) and (self.map_origin is not None) and (not old_map_origin == self.map_origin):
            for index, (y, x) in enumerate(self.waypoints):
                new_y = y + round((old_map_origin.y - self.map_origin.y) / map_res)
                new_x = x + round((old_map_origin.x - self.map_origin.x) / map_res)
                self.waypoints[index] = (new_y, new_x)
            print(f"Transformed Waypoints: {self.waypoints}")

    def odom_callback(self, msg):
        orientation_quat =  msg.pose.pose.orientation
        # updates robot roll, pitch and yaw
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    # mainly to check for obstacles and initiate obstacle avoidance    
    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges) # create numpy array of laser scans
        self.laser_range[self.laser_range==0] = np.nan # replace 0's with nan
        lr2i = np.nanargmin(self.laser_range)     
        # if closest point is less than stop distance, initiate obstacle avoidance sequence and set obstacle to true to break while loops in purepursuit/rotatebot
        if (self.laser_range[lr2i] < stop_distance_from_obstacle) and (lr2i < 135 or 225 < lr2i):
            print('OBSTACLE DETECTED FRONT')
            # convert lr2i to real world angle
            rot_angle = lr2i + math.degrees(self.yaw)
            rot_angle = rot_angle % 360
            # If the angle is greater than 180, subtract 360 to bring it to the range [-180, 180)
            if rot_angle > 180:
                rot_angle -= 360
            print(f"Obstacle Angle: {rot_angle}")
            self.obstacle_angle = rot_angle
            self.obstacle = True
            self.direction = 2 # front face and reverse from obstacle
        elif (self.laser_range[lr2i] < stop_distance_from_obstacle_behind) and (135 <= lr2i <= 225):
            print('OBSTACLE DETECTED BACK')
            # convert lr2i to real world angle
            rot_angle = lr2i + math.degrees(self.yaw) + 180
            rot_angle = rot_angle % 360
            # If the angle is greater than 180, subtract 360 to bring it to the range [-180, 180)
            if rot_angle > 180:
                rot_angle -= 360
            print(f"Obstacle Angle: {rot_angle}")
            self.obstacle_angle = rot_angle
            self.obstacle = True
            self. direction = 1 # backface and drive forward away from obstacle
        else:
            self.obstacle = False

    def szs_callback(self, msg):
        print('SURVIVOR ZONE SEQUENCE ACTIVE')
        self.szsactive = msg.data

    # creates path and then reduce path to waypoints
    def createpath(self):
        start = (self.grid_y, self.grid_x)
        end = (self.decisionpoint[0], self.decisionpoint[1])
        print(f"Start: {start}")
        print(f"End: {end}")
        grid = self.odata
        astar = AStar(grid, start, end)
        self.waypoints = []
        counter = 0
        for index, (y, x) in enumerate(astar.path):
            if (counter == waypoint_gap):
                self.waypoints.append((y, x))
                counter = 0
            counter += 1
        self.waypoints.pop() # remove last waypoint so robot does not go too near wall if last way point is wall
        print(f"Waypoints: {self.waypoints}")

    # rotate towards next waypoint and move straight until waypoint reached
    # repeat until no more waypoints left or within dp range
    # may break out of while loop if obstacle detected 
    def purepursuit(self):
        while len(self.waypoints) > 0 and self.pathfinderactive:
            rclpy.spin_once(self)
            currentwaypoint = self.waypoints.pop(0) # pop the next waypoint off list
            print(f"Currentwaypoint: {currentwaypoint}")
            
            # calculate angle from bot to next waypoint
            diff_y = currentwaypoint[0] - self.grid_y
            diff_x = currentwaypoint[1] - self.grid_x
            final_angle_in_degrees = None
            if diff_y != 0:
                angle_in_radians = math.atan(abs(diff_x/diff_y)) # calculate special angle from robot to next waypoint
                angle_in_degrees = math.degrees(angle_in_radians)
                final_angle_in_degrees = 0
                if diff_y > 0 and diff_x >= 0:
                    final_angle_in_degrees = 90-angle_in_degrees
                if diff_y > 0 and diff_x < 0:
                    final_angle_in_degrees = 90+angle_in_degrees
                if diff_y < 0 and diff_x >= 0:
                    final_angle_in_degrees = (90-angle_in_degrees)*(-1)
                if diff_y < 0 and diff_x < 0:
                    final_angle_in_degrees = (90+angle_in_degrees)*(-1)
            else:
                if diff_x > 0:
                    final_angle_in_degrees = 0
                if diff_x < 0:
                    final_angle_in_degrees = 180

            # rotate to that direction
            print(f"Desired Angle Inputted: {final_angle_in_degrees}")
            self.rotatebot(float(final_angle_in_degrees), False)

            # move robot forward after rotating to correct angle
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            time.sleep(0.5)
            self.publisher_.publish(twist)
            print('moving forward')

            # check if robot has reached waypoint
            reachedwaypoint = False
            while not reachedwaypoint:
                y_dist_to_wp = np.abs(self.grid_y - currentwaypoint[0])
                x_dist_to_wp = np.abs(self.grid_x - currentwaypoint[1])
                if (y_dist_to_wp < stop_distance_from_wp) and (x_dist_to_wp < stop_distance_from_wp):
                    reachedwaypoint = True
                rclpy.spin_once(self)
            # if robot detects obstacle, reacheddp, or szsactive, exit pure pursuit
                if self.obstacle is True or self.reacheddp or self.szsactive:
                    print('break 1')
                    break
            if self.obstacle is True or self.reacheddp or self.szsactive:
                print('break 2')
                break
        # stop robot before ending pure pursuit
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        if self.obstacle is True:
            self.obstacleavoidance()

        # if no more waypoints, take it that reacheddp
        # if szsactive, say reacheddp so that can killpathfinder 
        if len(self.waypoints) == 0 or self.szsactive:
            self.reacheddp = True
        self.killpathfinder()

    # rotates robot towards input angle
    def rotatebot(self, rot_angle, obstacleavoidance):
        twist = Twist()
        current_yaw = self.yaw # get current yaw angle
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))        
        c_target_yaw = complex(math.cos(math.radians(rot_angle)),math.sin(math.radians(rot_angle))) # convert to complex notation
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        c_change = c_target_yaw / c_yaw # divide the two complex numbers to get the change in direction
        c_change_dir = np.sign(c_change.imag) # get the sign of the imaginary component to figure out which way we have to turn
        twist.linear.x = 0.0 # set linear speed to zero so the TurtleBot rotates on the spot
        twist.angular.z = c_change_dir * rotatechange # set the direction to rotate
        self.publisher_.publish(twist) # start rotation

        # we will use the c_dir_diff variable to see if we can stop rotating
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        c_dir_diff = c_change_dir
        while(c_change_dir * c_dir_diff > 0):
            rclpy.spin_once(self) # allow the callback functions to run
            current_yaw = self.yaw     
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw)) # convert the current yaw to complex form
            c_change = c_target_yaw / c_yaw # get difference in angle between current and target
            c_dir_diff = np.sign(c_change.imag) # get the sign to see if we can stop
            # stop rotatebot if obstacle detected, unless rotatebot was called from obstacleavoidance
            if (self.obstacle is True and not obstacleavoidance) or self.szsactive:
                break
            
        # stop rotation
        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    # targetlock mode: clears waypoints and sets pathfinderactive to false if reacheddp, else pathfinderactive still true, createpath and purepursuit towards same dp
    # non targetlock mode: clears waypoints and sets pathfinderactive to false 
    def killpathfinder(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        # targetlock mode
        if targetlock is True:
            self.waypoints = []
            # reacheddp is True when either within dp distance or no more waypoints
            if self.reacheddp:
                print('killpathfinder')
                self.pathfinderactive = False
                msg = Bool()
                msg.data = self.pathfinderactive
                self.pathfinderactive_publisher_.publish(msg)
                self.reacheddp = False # reset reacheddp since exiting pathfinder
                self.decisionpoint = None
            else: 
                print('recalibrating path')
        # non targetlock mode    
        else:
            self.waypoints = []
            print('killpathfinder')
            self.pathfinderactive = False
            msg = Bool()
            msg.data = self.pathfinderactive
            self.pathfinderactive_publisher_.publish(msg)
            self.reacheddp = False # reset reacheddp since exiting pathfinder
            self.decisionpoint = None

    # stop the robot immediately, rotate robot towards closest point and reverse away slightly
    def obstacleavoidance(self):
        print('obstacle avoidance')
        twist = Twist()
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.rotatebot(float(self.obstacle_angle), True) # rotate to face/backface direction of closest point
        if self.direction == 2: # if obstacle is from the front, reverse away
            while True:
                twist.linear.x = -speedchange # reverse away from closest point slightly
                self.publisher_.publish(twist)
                time.sleep(0.3) # best timing is 1ish
                twist.linear.x = 0.0 # stop robot
                self.publisher_.publish(twist)
                print('reversing from obstacle')
                rclpy.spin_once(self)
                if not self.obstacle:
                    break
        if self.direction == 1: # if obstacle is from the back, move forward awway
            while True:
                twist.linear.x = speedchange # reverse away from closest point slightly
                self.publisher_.publish(twist)
                time.sleep(0.3) # best timing is 1ish
                twist.linear.x = 0.0 # stop robot
                self.publisher_.publish(twist)
                print('moving forward from obstacle')
                rclpy.spin_once(self)
                if not self.obstacle:
                    break
        self.direction = 0 # reset direction to neutral

        

def main(args=None): 
    rclpy.init(args=args)
    pathfinder = Pathfinder()
    # create matplotlib figure
    plt.ion()
    plt.show()

    while True:
        rclpy.spin_once(pathfinder)
        if (not pathfinder.pathfinderactive) and (pathfinder.decisionpoint is not None): 
            pathfinder.pathfinderactive = True
            msg = Bool()
            msg.data = pathfinder.pathfinderactive
            pathfinder.pathfinderactive_publisher_.publish(msg)
            while pathfinder.pathfinderactive is True:
                print('Pathfinderactive')
                pathfinder.createpath()
                pathfinder.purepursuit()
        # freeze pathfinder while szsactive
        while pathfinder.szsactive:
            rclpy.spin_once(pathfinder)
            print('stuck in szs')
                

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pathfinder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()