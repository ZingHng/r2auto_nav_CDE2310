import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
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
stop_distance_from_obstacle = 0.18 # distance to killpathfinder
waypoint_gap = 4 # number of grids between pure pursuit waypoints
rotatechange = 0.2 # speed of rotation
speedchange = 0.1 # speed of linear movement

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
        
    def build_astar_costmap(self):
        for grid_y in range(self.rows):  # Iterate through rows
            for grid_x in range(self.cols):  # Iterate through columns
                x_diff = np.abs(grid_x - self.start[1])
                y_diff = np.abs(grid_y - self.start[0])
                g_cost = np.sqrt(x_diff**2 + y_diff**2)
                x_diff = np.abs(grid_x - self.end[1])
                y_diff = np.abs(grid_y - self.end[0])
                h_cost = np.sqrt(x_diff**2 + y_diff**2)
                f_cost = g_cost + h_cost
                self.astar_costmap[grid_y][grid_x] = round(f_cost*10)
    def build_wallprox_costmap(self):
        for grid_y in range(self.rows):  # Iterate through rows
            for grid_x in range(self.cols):  # Iterate through columns
                if self.grid[grid_y][grid_x] == 3: # If the current cell is a wall, it is given cost of 10000 and surrounding cells are marked
                    self.wallprox_costmap[grid_y][grid_x] = 10**10
                    for dy in range(-8, 9):  # Look within 8 cells distance
                        for dx in range(-8, 9):
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
                               
    def print(self):
        pathmap = self.grid
        pathmap[self.start[0]][self.start[1]] = 0
        for (y, x) in self.path:
            pathmap[y, x] = 0

        img = Image.fromarray(pathmap) # create image from 2D array using PIL

        # show the image using grayscale map
        plt.imshow(img, cmap='magma', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(1)
        

    def dijkstra(self, grid, start, end):
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        rows, cols = len(grid), len(grid[0])
        
        # Priority queue with (cost, row, col)
        pq = [(grid[start[0]][start[1]], start[0], start[1])]
        
        # Distance matrix initialized to infinity
        dist = [[float('inf')] * cols for _ in range(rows)]
        dist[start[0]][start[1]] = grid[start[0]][start[1]]
        
        # Previous matrix to reconstruct the path
        prev = [[None] * cols for _ in range(rows)]
        
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


class Pathfinder(Node):
    def __init__(self):
        super().__init__('pathfinder')
        self.pathfinderactive = False
        self.dp_subscription = self.create_subscription(
            Point,
            'decisionpoint',
            self.dp_callback,
            10)
        self.dp_subscription # prevent unused variable warning
        self.decisionpoint = None
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
        #add cmd_vel publisher
        self.pathfinderactive_publisher_ = self.create_publisher(Bool, 'pathfinderactive', 10) #pathfinderactive publisher
        self.map_origin = None
        self.waypoints = None
        self.reachedwaypoint = False



        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        
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

    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        #print(f"Self.yaw: {self.yaw}")

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)

        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        lr2i = np.nanargmin(self.laser_range)
        # print(lr2i)
        # np.savetxt('laser.txt', self.laser_range)
        
        # log the info
        if self.laser_range[lr2i] < stop_distance_from_obstacle:
            print('OBSTACLE DETECTED')
            self.obstacleavoidance()

    def dp_callback(self, msg):
        print('dpcallback')
        self.decisionpoint = (msg.y, msg.x)
        


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
        '''
        self.odata[self.grid_y][self.grid_x] = 0 # set current robot location to 0 to see on the matplotlib
        img = Image.fromarray(self.odata) # create image from 2D array using PIL

        # show the image using grayscale map
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)
        '''
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
        
        # kill pathfinder if decision point reached
        if (self.decisionpoint is not None):
            y_dist_to_dp = np.abs(self.grid_y - self.decisionpoint[0])
            x_dist_to_dp = np.abs(self.grid_x - self.decisionpoint[1])
            if (y_dist_to_dp < stop_distance_from_dp) and (x_dist_to_dp < stop_distance_from_dp):
                print(y_dist_to_dp)
                print(x_dist_to_dp)
                self.killpathfinder() # stops pathfinder      

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

    def killpathfinder(self):
        self.waypoints = []
        if (self.reachedwaypoint == True):
            print('killpathfinder')
            self.pathfinderactive = False            
            msg = Bool()
            msg.data = self.pathfinderactive
            self.pathfinderactive_publisher_.publish(msg)
        else:
            print('loading new path')

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
        self.waypoints.pop()
        print(f"Waypoints: {self.waypoints}")

    def purepursuit(self):
        while len(self.waypoints) > 0 and self.pathfinderactive:
            rclpy.spin_once(self)

            currentwaypoint = self.waypoints.pop(0)
            print(f"Currentwaypoint: {currentwaypoint}")

            diff_y = currentwaypoint[0] - self.grid_y
            print(diff_y)
            diff_x = currentwaypoint[1] - self.grid_x
            print(diff_x)
            if diff_y != 0:
                angle_in_radians = math.atan(abs(diff_x/diff_y)) # calculate special angle from robot to next waypoint
                angle_in_degrees = math.degrees(angle_in_radians) #convert to degrees
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
            self.rotatebot(float(final_angle_in_degrees))

            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more reliably with this
            time.sleep(0.5)
            self.publisher_.publish(twist)
            print('moving forward')

            while (not reachedwaypoint) and self.pathfinderactive:
                y_dist_to_wp = np.abs(self.grid_y - currentwaypoint[0])
                x_dist_to_wp = np.abs(self.grid_x - currentwaypoint[1])
                if (y_dist_to_wp < stop_distance_from_wp) and (x_dist_to_wp < stop_distance_from_wp):
                    self.reachedwaypoint = True
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                rclpy.spin_once(self)
        self.killpathfinder()

    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # convert to complex notation
        c_target_yaw = complex(math.cos(math.radians(rot_angle)),math.sin(math.radians(rot_angle)))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
       
    
        while(c_change_dir * c_dir_diff > 0) and self.pathfinderactive:
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            #self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            #self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def obstacleavoidance(self):
        # retreat from obstacle slightly
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        twist.linear.x = -speedchange
        self.publisher_.publish(twist)
        time.sleep(0.5)
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        # kill current pathfinder process
        self.killpathfinder()
        # purepursuit() will end with self.reachedwaypoint = False
        # in main, new path will be created to the same deicison point and pure pursuit will follow


def main(args=None): 
    rclpy.init(args=args)
    pathfinder = Pathfinder()
    # create matplotlib figure
    plt.ion()
    plt.show()

    while True:
        # olddp = pathfinder.decisionpoint
        rclpy.spin_once(pathfinder)
        if (not pathfinder.pathfinderactive) and (pathfinder.decisionpoint is not None): 
            pathfinder.reachedwaypoint = False
            while (pathfinder.reachwaypoint == False):
                pathfinder.pathfinderactive = True
                msg = Bool()
                msg.data = pathfinder.pathfinderactive
                self.pathfinderactive_publisher_.publish(msg)
                print('Pathfinderactive')
                pathfinder.createpath()
                pathfinder.purepursuit()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pathfinder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
