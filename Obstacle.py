#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #


import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import smbus
# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)
time.sleep(1)

def getAndUpdateColour(): #This function reads data from the RGB sensor via the i2c protocol, and decides which color is dominant
    #while True:
        # Read the data from the sensor
        # Convert the data to green, red and blue int values
        # Insert code here
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        green = data[1] + data[0]/256
        red = data[3] + data[2]/256
        blue = data[5] + data[4]/256

        colour = ""
        if green > red and green > blue:
            colour = "Green"
        elif blue > red:
            colour = "Blue"
        else:
            colour = "Red"
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)" % (red, green, blue))
        print("The colour is " + colour)
        return colour

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.40
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR + 0.1

class Obstacle(): #We define the obstacle class, encapsulating all its behavior and methods related to obstacle detection and avoidance
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self): #Function to handle all the data from the message from the LIDAR
        scan = rospy.wait_for_message('scan', LaserScan) #Wait for "scan" message from the LaserScan
        scan_filter = []
        front = [] #We seperate the data into three different directions, front, left and right
        left = []
        right = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 3            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 3:
            for i in range (15):
                    front.append(scan.ranges[i])
                    front.append(scan.ranges[359-i])   
                    left.append(scan.ranges[315+i])
                    left.append(scan.ranges[314-i])
                    right.append(scan.ranges[45+i])
                    right.append(scan.ranges[44-i])

            scan_filter.append(front)
            scan_filter.append(left)
            scan_filter.append(right)

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
           
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)
        

        return scan_filter

    def obstacle(self): #Primary function of our Obstacle class, which is used to navigate our robot
        twist = Twist()
        turtlebot_moving = True
        average_speed = []
        victim_count = 0
        colission_count = 0
        last_run_time_rgb = 0
        last_run_time_col = 0
        delay_col = 2
        delay_rgb = 3

        runtime = time.time() + 120

        while not rospy.is_shutdown() and time.time() < runtime:
            all_dist = self.get_scan()
            front_min_distance, i_f = min_org(all_dist[0],SAFE_STOP_DISTANCE+0.001)
            right_min_distance, _ = min_org(all_dist[1],SAFE_STOP_DISTANCE+0.001)
            left_min_distance, _ = min_org(all_dist[2],SAFE_STOP_DISTANCE+0.001)
            
                #Depending on the minimum distance to each different side, it sets the linear velocities of the robot
            if (0.000 < right_min_distance < 0.130) or (0.000 < left_min_distance < 0.130) or (0.000 < front_min_distance < 0.130):

                if left_min_distance <= right_min_distance:
                    twist.angular.z = -1.5
                    twist.linear.x = 0
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('You spin me right round baby right round')
                else:
                    twist.angular.z = 1.5
                    twist.linear.x = 0
                    self._cmd_pub.publish(twist)
                    rospy.loginfo('You spin me left round baby left round')
                
                average_speed.append(0)
                if time.time() > last_run_time_col + delay_col:
                    colission_count += 1
                    rospy.loginfo('Colissions reggistered: %f', colission_count)
                    last_run_time_col = time.time()

            elif 0.00 < front_min_distance < STOP_DISTANCE:

                # Center is closest
                if front_min_distance < left_min_distance and front_min_distance < right_min_distance and (0.00 < front_min_distance < STOP_DISTANCE):
                    if i_f % 2 == 0:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = -2
                        self._cmd_pub.publish(twist)
                        #turtlebot_moving = False
                        rospy.loginfo('Stop! Center distance of the obstacle, driving right : %f', front_min_distance)
                    else:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 2
                        self._cmd_pub.publish(twist)
                        #turtlebot_moving = False
                        rospy.loginfo('Stop! Center distance of the obstacle, driving left : %f', front_min_distance)
                    rospy.loginfo('%f',i_f)

                # Right side is closest
                elif right_min_distance < left_min_distance and right_min_distance < front_min_distance and (0.00 < right_min_distance < STOP_DISTANCE):
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 3 * (right_min_distance/front_min_distance) 
                    self._cmd_pub.publish(twist)
                    average_speed.append((1-(right_min_distance/front_min_distance))*LINEAR_VEL)
                    #turtlebot_moving = False
                    rospy.loginfo('Stop! Driving left. ' + 'Distance of the obstacle : %f', right_min_distance)

                # Left side is closest 
                elif left_min_distance < front_min_distance and left_min_distance < right_min_distance and (0.00 < left_min_distance < STOP_DISTANCE):
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = -3 * (left_min_distance/front_min_distance)
                    self._cmd_pub.publish(twist)
                    average_speed.append((1-(right_min_distance/front_min_distance))*LINEAR_VEL)
                    #turtlebot_moving = False
                    rospy.loginfo('Stop! Driving right. ' + 'Distance of the obstacle : %f', left_min_distance)

            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True

                average_speed.append(LINEAR_VEL)
                rospy.loginfo('We crusin with a distance of %f', front_min_distance)
            
            if len(average_speed) != 0:
                avspeed = sum(average_speed)/len(average_speed)
                rospy.loginfo('Average speed: %f', avspeed)

                #When
            if time.time() > last_run_time_rgb + delay_rgb:
                if getAndUpdateColour() == 'Red':        #when it detects red, it increases a victim count, assuming we found a "victim"
                    victim_count += 1
                    rospy.loginfo('Victims found: %f', victim_count)
                    last_run_time_rgb = time.time()
        rospy.loginfo('We found %f victims', victim_count)

def min_org(l,a):
    min = a
    min_i = 0
    for i in range(len(l)):
        if l[i] != 0.0000 and l[i] < min:
            min = l[i]
            min_i = i
    return min, min_i

def main():
    rospy.init_node('turtlebot3_obstacle')

    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

