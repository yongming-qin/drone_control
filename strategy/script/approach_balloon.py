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
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import PositionTarget


from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

import math

class MavController:
    """
    """
    def __init__(self):
        rospy.init_node("mav_control_node", anonymous=True)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.cmd_raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        
        # mode 0 = STABILIZED, mode 4 = GUIDED, mode 9 = LAND
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.takeoff_service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

        self.pose = Pose()
        self.timestamp = rospy.Time()

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
       
        angles = tf.transformations.euler_from_quaternion([msg_orientation.x,msg_orientation.y,msg_orientation.z,msg_orientation.w])

        rospy.loginfo("pose: %f, %f, %f, %f, %f, %f",
                self.pose.position.x, self.pose.position.y, self.pose.position.z,
                angles[0], angles[1], angles[2])

    def goto_position_by_vel(self, x, y, z, vyaw):
        while not rospy.is_shutdown():
            dx = x - self.pose.position.x
            dy = y - self.pose.position.y
            dz = z - self.pose.position.z
            if (math.fabs(dx) < 0.2 and math.fabs(dy) < 0.2 and math.fabs(dz) < 0.2):
                self.set_vel(0, 0, 0, avz=0)
                return(True)
            def velocity(input):
                k = 0.5
                limit = 1
                if math.fabs(input) >= 0.2:
                    output = k*input if math.fabs(k*input) < limit else math.copysign(limit,k*input)
                    # print("velocity: ", output)
                    return(output)
                else:
                    return 0

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
        self.c.goto_position_by_vel(0,      0,          3,  0.2)
        self.c.goto_position_by_vel(15,     0,          3,  0.2)
        self.c.goto_position_by_vel(15,     -10,        3,  0.2)
        self.c.goto_position_by_vel(5,      -10,        3,  0.2)
        self.c.goto_position_by_vel(0,      0,          3,  0.2)

        

    def takeoff_and_search(self):
        """
        search #TODO
        """
        alt = 2.5
        print("Takeoff " + str(alt))
        self.c.takeoff(alt)
        rospy.sleep(5)

        



if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        print("x: " + str(x) + " y: " + str(y) + " z: " + str(z))


    approach = Behavior()

    # 8 shape flying
    #approach.eight_vel()

    approach.search()


