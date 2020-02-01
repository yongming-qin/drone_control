#!/usr/bin/env python
"""
Now I finished using zed to sensor the balloon's position.
This code is to let the drone fly to the balloon.
Yongming Qin
2020/01/12
"""

from __future__ import print_function
import sys
import rospy
import tf
import math
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from geometry_msgs.msg import Point, Vector3Stamped
from mavros_msgs.msg import PositionTarget

from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

from strategy.srv import ChangeState, ChangeStateResponse

import math

class MavController:
    """
    """
    def __init__(self):
        rospy.init_node("mav_control_node", anonymous=True)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        #QIN: go to target and face toward the target using setpoint_velocity
        rospy.Subscriber("/estimated_position", Vector3Stamped, self.approach_callback)

        
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        #QIN can last for 2 seconds. This saves my effort to publish in a high rate like 100hz
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.cmd_raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        
        # mode 0 = STABILIZED, mode 4 = GUIDED, mode 9 = LAND
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.takeoff_service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

        self.change_state_service = rospy.Service("change_state", ChangeState, self.change_state)

        self.pose = Pose()
        self.angles = [0, 0, 0]
        self.timestamp = rospy.Time()

        self.STATE = "idle"
        # [x,y,z,vz], vz = rotation_speed
        height = 3
        self.route = [[0,0,height], [35,0,height], [35,-10,height],\
                      [5,-10,height],[5,-20,height],\
                      [35,-20,height],[35,-30,height],\
                      [5,-30,height],[5,-40,height],\
                      [35,-40,height],\
                      [0,0,height]]

        while not rospy.is_shutdown():
            pass

    def change_state(self, req):
        self.STATE = req.state
        return ChangeStateResponse("processed")


    def goto_raw(self, x, y, z):
        cmd_raw = PositionTarget()

        # uint8 coordinate_frame
        # uint8 FRAME_LOCAL_NED = 1
        # uint8 FRAME_LOCAL_OFFSET_NED = 7
        # uint8 FRAME_BODY_NED = 8
        # uint8 FRAME_BODY_OFFSET_NED = 9 # relative to the vehicle's current position and heading
        cmd_raw.coordinate_frame = 1 

        # uint16 type_mask
        # uint16 IGNORE_PX = 1 # Position ignore flags
        # uint16 IGNORE_PY = 2
        # uint16 IGNORE_PZ = 4
        # uint16 IGNORE_VX = 8 # Velocity vector ignore flags
        # uint16 IGNORE_VY = 16
        # uint16 IGNORE_VZ = 32
        # uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
        # uint16 IGNORE_AFY = 128
        # uint16 IGNORE_AFZ = 256
        # uint16 FORCE = 512 # Force in af vector flag
        # uint16 IGNORE_YAW = 1024
        # uint16 IGNORE_YAW_RATE = 2048
        cmd_raw.type_mask = 3576
        # Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        # Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
        # Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)

        position = Point()
        position.x = float(x); position.y = float(y); position.z = float(z)
        cmd_raw.position = position

        self.cmd_raw_pub.publish(cmd_raw)


    def pose_callback(self, msg):
        """
        Handle local position information
        """
        self.timestamp = msg.header.stamp
        self.pose = msg.pose
        msg_orientation = msg.pose.orientation
       
        self.angles = tf.transformations.euler_from_quaternion([msg_orientation.x,msg_orientation.y,msg_orientation.z,msg_orientation.w])

        rospy.loginfo("state: %s", self.STATE)
        rospy.loginfo("pose: %f, %f, %f, %f, %f, %f",
                self.pose.position.x, self.pose.position.y, self.pose.position.z,
                self.angles[0], self.angles[1], self.angles[2])

        # Search the balloons
        if self.STATE == "search":
            if len(self.route) >= 1:
                if self.go_to_position_nonholonomic(self.route[0][0], self.route[0][1], self.route[0][2]):
                    del self.route[0]
            else:
                print("Search Finished, visited all the waypoints")
        


    def approach_callback(self, msg):
        if self.STATE != "approach":
            return
        x = msg.vector.x; y = msg.vector.y; z = msg.vector.z
        self.go_to_position_nonholonomic(x, y, z)

    def go_to_position_nonholonomic(self, x, y, z):
        reach_radius = 0.5
        def angle_difference(direction, direction_goal):
            dif = direction - direction_goal
            while dif > math.pi:
                dif -= 2*math.pi
            while dif < -math.pi:
                dif += 2*math.pi
            return dif

        def linear_vel(input):
            k = 0.5
            limit = 1
            if math.fabs(input) >= reach_radius:
                output = k*input if math.fabs(k*input) < limit else math.copysign(limit,k*input)
                # print("velocity: ", output)
                return(output)
            else:
                return(0)
        
        def angular_vel(input):
            reach_angle = 0.2 #  degrees
            k = 2
            limit = 0.3
            if math.fabs(input) >= reach_angle:
                output = k*input if math.fabs(k*input) < limit else math.copysign(limit,k*input)
                return(output)
            else:
                return(0)

        dx = x - self.pose.position.x
        dy = y - self.pose.position.y
        dz = z - self.pose.position.z
        distance = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
        sin_direction = dy / distance
        cos_direction = dx / distance
        direction = math.atan2(sin_direction, cos_direction)
        #QIN approximatedly control the yaw, which makes the target is in the view
        error_direction = angle_difference(direction, self.angles[2])
        print("error_direction: ", error_direction)

        if (math.fabs(dx) < reach_radius and math.fabs(dy) < reach_radius and math.fabs(dz) < reach_radius):
            self.set_vel(0, 0, 0, avz=0)
            return(True)
        else:
            #DEBUG: for test
            self.set_vel(linear_vel(dx), linear_vel(dy), linear_vel(dz), avz=angular_vel(error_direction))
            return(False)

    def go_to_position_holonomic(self, x, y, z, vyaw=0):
        reach_radius = 0.5
        dx = x - self.pose.position.x
        dy = y - self.pose.position.y
        dz = z - self.pose.position.z
        if (math.fabs(dx) < reach_radius and math.fabs(dy) < reach_radius and math.fabs(dz) < reach_radius):
            self.set_vel(0, 0, 0, avz=0)
            return

        def velocity(input):
            k = 0.5
            limit = 1
            if math.fabs(input) >= reach_radius:
                output = k*input if math.fabs(k*input) < limit else math.copysign(limit,k*input)
                # print("velocity: ", output)
                return(output)
            else:
                return(0)
        self.set_vel(velocity(dx), velocity(dy), velocity(dz), avz=vyaw)
        

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Set to guided mode, arm the throttle, takeoff to a few meters
        """
        # Set to guided mode
        #QIN safer to use rc controller to change the mode
        #mode_resp = self.mode_service(custom_mode="4"); rospy. sleep(1)

        # Arm throttle
        self.arm(); rospy.sleep(1)

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly
        land and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()
    
    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

class Behavior():
    def __init__(self):
        self.c = MavController()
        rospy.sleep(1)

    def eight_vel(self):
        """
        8 figure flying, two circles
        """
        # alt = 3.5
        # print("Takeoff " + str(alt))
        # self.c.takeoff(alt)
        # rospy.sleep(5)

        t_start = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - t_start < rospy.Duration(60):
            self.c.set_vel(0,0,0,avz=0.1)

        speed = 1 # will be constant. circle
        period = 20.0 # one circle of the 8 shape
        # Then diameter = speed * period / math.pi

        def circle_flying(speed, period, is_clockwise):
            print("clockwise" if is_clockwise else "counterclockwise" + " flying")
            t_start = rospy.Time.now()
            t = rospy.Time.now() - t_start
            while not rospy.is_shutdown() and t < rospy.Duration(period):
                t2 = t.to_sec() * 2*math.pi/period
                multiplier = 1 if is_clockwise else -1
                self.c.set_vel(speed * multiplier * math.sin(t2), speed * math.cos(t2), 0)
                t = rospy.Time.now() - t_start

        for _ in range(2):
            circle_flying(speed, period, True)
            circle_flying(speed, period, False)

        # print("Landing")
        # self.c.land()

    def search(self):
        rotation_speed = 0.2
        self.c.go_to_position_holonomic(0,      0,          3,  0)
        self.c.go_to_position_holonomic(15,     0,          3,  rotation_speed)
        self.c.go_to_position_holonomic(15,     -10,        3,  rotation_speed)
        self.c.go_to_position_holonomic(5,      -10,        3,  rotation_speed)
        self.c.go_to_position_holonomic(0,      0,          3,  rotation_speed)

        

    def takeoff_and_search(self):
        """
        search #TODO
        """
        alt = 2.5
        print("Takeoff " + str(alt))
        self.c.takeoff(alt)
        rospy.sleep(5)

        



if __name__ == "__main__":
    # if len(sys.argv) == 4:
    #     x = float(sys.argv[1])
    #     y = float(sys.argv[2])
    #     z = float(sys.argv[3])

    #     print("x: " + str(x) + " y: " + str(y) + " z: " + str(z))


    approach = MavController()



