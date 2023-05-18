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
#import smbus
# Get I2C bus
##bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
##bus.write_byte_data(0x44, 0x01, 0x05)

##time.sleep(1)

#print("Reading colour values and displaying them in a new window\n")

#def getAndUpdateColour():
#    while True:
#        # Read the data from the sensor
#       # Convert the data to green, red and blue int values
#        # Insert code here
#        data = bus.read_i2c_block_data(0x44, 0x09, 6)
#        green = data[1] + data[0]/256
#        red = data[3] + data[2]/256
#        blue = data[5] + data[4]/256

#        colour = ""
#        if green > red and green > blue:
#            colour = "green"
#        elif blue > red:
#            colour = "Blue"
#        else:
#            colour = "Red"
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
#        print("RGB(%d %d %d)" % (red, green, blue))
#        print("The colour is " + colour)
#        time.sleep(2)

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.20
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR + 0.1

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        front = []
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

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            all_dist = self.get_scan()
            front_min_distance = min_org(all_dist[0],STOP_DISTANCE+0.01)
            left_min_distance = min_org(all_dist[2],STOP_DISTANCE+0.01)
            right_min_distance = min_org(all_dist[1],STOP_DISTANCE+0.01)

            if 0 < front_min_distance < SAFE_STOP_DISTANCE:
                # Center is closest
                if front_min_distance < left_min_distance and front_min_distance < right_min_distance and (0 < front_min_distance < STOP_DISTANCE):
                    twist.linear.x = 0.05
                    twist.angular.z = 2
                    self._cmd_pub.publish(twist)
                    #turtlebot_moving = False
                    rospy.loginfo('Stop! Center distance of the obstacle, driving left : %f', front_min_distance)
            
                # Right side is closest
                elif right_min_distance < left_min_distance and right_min_distance < front_min_distance and (0 < right_min_distance < STOP_DISTANCE):
                    twist.linear.x = 0.1
                    twist.angular.z = 2
                    self._cmd_pub.publish(twist)
                    #turtlebot_moving = False
                    rospy.loginfo('Stop! Driving left. ' + 'Distance of the obstacle : %f', right_min_distance)

                # Left side is closest 
                elif left_min_distance < front_min_distance and left_min_distance < right_min_distance and (0 < left_min_distance < STOP_DISTANCE):
                    twist.linear.x = 0.1
                    twist.angular.z = -2
                    self._cmd_pub.publish(twist)
                    #turtlebot_moving = False
                    rospy.loginfo('Stop! Driving right. ' + 'Distance of the obstacle : %f', left_min_distance)
                
                # If no distance is critical, we drive more carefull now.
                else:
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True
                    rospy.loginfo('We crusin with, but lookin out with a distance of %f', front_min_distance)

            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('We crusin with a distance of %f', front_min_distance)

def min_org(l,a):
    min = a
    for i in range(len(l)):
        if l[i] != 0 and l[i] < min:
            min = l[i]
    return min

def main():
    rospy.init_node('turtlebot3_obstacle')
    #print("Reading colour values and displaying them in a new window\n")
    #getAndUpdateColour()
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
