#!/usr/bin/env python
"""
Now I finished using zed to sensor the ballon's position.
Compare with the version of 01/12, yolo is retrained with figures of the field
and the point of balloon is transformed into the /map frame.
I plan to use LOCAL_NED for position and velocity control.
The drone needs rotation to make sure the camera can cover the target.
But actually it desn't need to rotate to catch the ball or pop the balloon.
Yongming Qin
2020/01/28
"""

import sys
from math import pow, sqrt
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from geometry_msgs.msg import Vector3Stamped
from darknet_ros_msgs.msg import BalloonPositions

from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

class MavController:
    def __init__(self):
        rospy.init_node("mav_control_node", anonymous=True)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/darknet_ros/balloon_positions", BalloonPositions, self.balloon_positions_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.cmd_raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        self.position_pub = rospy.Publisher("/estimated_position", Vector3Stamped, queue_size=1)

        # mode 0 = STABILIZED, mode 4 = GUIDED, mode 9 = LAND
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.takeoff_service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

        self.pose = Pose()
        self.timestamp = rospy.Time()

        # Estimation of balloons positions
        self.points = []
        self.count = 0
        self.popped_positions = []


        while not rospy.is_shutdown():
            pass

        
    def pose_callback(self, msg):
        pass
    

    def balloon_positions_callback(self, msg):
        same_point_radius = 2
        # print(type(msg.balloon_positions[0].point))
        for new_position in enumerate(msg.balloon_positions):
            #QIN Got the information by printing the type and data structure
            # new_position is a 2d tuple
            distance_to_drone = new_position[1].distance_to_drone
            ros_point = new_position[1].point
            new_point = [ros_point.x, ros_point.y, ros_point.z]
            # exclude some points
            if self.ignore_point(new_point, distance_to_drone):
                continue
            
            # print("new point: ", new_point)

            existed = False
            for old_point in self.points:
                # close to a previous point
                #TODO update the closest point
                if self.distance_points(new_point, old_point["point"]) < same_point_radius:
                    self.update_point(old_point["point"], new_point)
                    old_point["count"] += 1
                    existed = True
                    break
            if existed:
                continue
            else: # add it to self.points
                self.points.append({"point": new_point, "count": 1})
        
        self.count += 1
        # print(len(self.points))
        if self.count % 20 == 1:
            for idx, point in enumerate( sorted(self.points, key=lambda val: val["count"], reverse=True) ):
                print("count: " + str(point["count"]), point["point"])
                if point["count"] > 8:
                    # print(point["count"], "\t", point["point"])
                    #QIN Only the point with the most count will be published
                    if idx == 0:
                        position = Vector3Stamped()
                        position.header = msg.header
                        position.vector.x = self.points[0]["point"][0]
                        position.vector.y = self.points[0]["point"][1]
                        position.vector.z = self.points[0]["point"][2]
                        self.position_pub.publish(position)
            print("----------------------------------------------")
        if self.count % 100 == 1:
            self.points = []
        

    def ignore_point(self, point, distance_to_drone):
        # Check if the balloon is already visited TODO: not for real competition
        threshold = 2
        for p in self.popped_positions:
            if self.distance_points(p, point) < threshold:
                return True

        # Check if the point is out of range, which will not be accurate
        max_range = 10
        if distance_to_drone > max_range:
            return True

        # Check if the point is out of area
        # if point[0] < -1 or point[0] > 35 or point[1] > 6 or point[1] < -30:
        #     return True

        return False

    def distance_points(self, point1, point2):
        return sqrt( pow((point1[0] - point2[0]), 2) + \
                     pow((point1[1] - point2[1]), 2) + \
                     pow((point1[2] - point2[2]), 2) )

    def update_point(self, old_point, new_point):
        old_point[0] += 0.2 * (new_point[0] - old_point[0])
        old_point[1] += 0.2 * (new_point[1] - old_point[1])
        old_point[2] += 0.2 * (new_point[2] - old_point[2])




if __name__ == "__main__":
    mav_controller = MavController()
    
    

    

