##
#
# Control a MAV via mavros
#
##

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool

from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

import math

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        if not rospy.is_shutdown():
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z

            quat = tf.transformations.quaternion_from_euler(ro, pi, ya + math.pi/2)

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.goto(pose)
            #print(quat)

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
        mode_resp = self.mode_service(custom_mode="4") # safer to use rc controller to change the mode
        rospy.sleep(1)

        # arm throttle
        self.arm()
        rospy.sleep(1)

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #TODO check failure

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

def simple_demo():
    """
    take off and land
    """
    c = MavController()
    rospy.sleep(5)
    alt = 5

    print("Takeoff " + str(alt))
    c.takeoff(alt)
    rospy.sleep(10)
    # c.goto_xyz_rpy(0,0,alt,0,0,0)
    rospy.sleep(10)

    print("Landing")
    c.land()

def shape_eight():
    """
    eight figure flying
    """
    c = MavController()
    rospy.sleep(1)
    alt = 3.5
    time = 7

    print("Takeoff " + str(alt))
    c.takeoff(alt)
    rospy.sleep(time)

    print("Waypoint: 0 0")
    c.goto_xyz_rpy(0.0,0.0,alt,0,0,-1*math.pi/4)
    rospy.sleep(time)
    print("Waypoint: 5 5")
    c.goto_xyz_rpy(5,5,alt,0,0,-1*math.pi/4)
    rospy.sleep(time)
    print("Waypoint: 10 0")
    c.goto_xyz_rpy(10,0,alt,0,0,math.pi/2)
    rospy.sleep(time)
    print("Waypoint: 5 -5")
    c.goto_xyz_rpy(5,-5,alt,0,0,3/2*math.pi)
    rospy.sleep(time)
    print("Waypoint: 0 0")
    c.goto_xyz_rpy(0,0,alt,0,0,3/2*math.pi)
    rospy.sleep(time)
    print("Waypoint: -5 5")
    c.goto_xyz_rpy(-5,5,alt,0,0,3/2*math.pi)
    rospy.sleep(time)
    print("Waypoint: -10 0")
    c.goto_xyz_rpy(-10,0,alt,0,0,1/2*math.pi)
    rospy.sleep(time)
    print("Waypoint: -5 -5")
    rospy.sleep(time)
    c.goto_xyz_rpy(-5,-5,alt,0,0,1/4*math.pi)
    print("Waypoint: return to 0 0")
    c.goto_xyz_rpy(0,0,alt,0,0,0)
    rospy.sleep(time)
    
    print("Landing")
    c.land()


def shape_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController()
    rospy.sleep(1)
    alt = 0.8

    print("Takeoff " + str(alt))
    c.takeoff(alt)
    rospy.sleep(3)
    c.goto_xyz_rpy(0,0,0.8,0,0,0)
    #rospy.sleep(3)

    print("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0,0.0,alt,0,0,-1/2*pi_2)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.4,0.0,alt,0,0,-1*pi_2)
    rospy.sleep(3)
    print("Waypoint 2: position control")
    c.goto_xyz_rpy(0.4,0.0,alt,0,0,0)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.4,0.4,alt,0,0,0)
    rospy.sleep(3)
    print("Waypoint 3: position control")
    c.goto_xyz_rpy(0.4,0.4,alt,0,0,pi_2)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.0,0.4,alt,0,0,pi_2)
    rospy.sleep(3)
    print("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0,0.4,alt,0,0,2*pi_2)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.0,0.0,alt,0,0,2*pi_2)
    rospy.sleep(3)
    #c.goto_xyz_rpy(0.0,0.0,alt,0,0,3*pi_2)
    #rospy.sleep(1)
    #c.goto_xyz_rpy(0.0,0.0,alt,0,0,4*pi_2)
    #rospy.sleep(2)

    #print("Velocity Setpoint 1")
    #c.set_vel(0,0.1,0)
    #rospy.sleep(5)
    #print("Velocity Setpoint 2")
    #c.set_vel(0,-0.1,0)
    #rospy.sleep(5)
    #print("Velocity Setpoint 3")
    #c.set_vel(0,0,0)
    #rospy.sleep(5)

    print("Landing")
    c.land()

def test_vel():
    c = MavController()
    rospy.sleep(1)
    alt = 3.5
    print("Takeoff " + str(alt))
    c.takeoff(alt)
    rospy.sleep(5)

    print("Velocity Setpoint 0.5 0 0")
    t = rospy.Time.now()
    while (not rospy.is_shutdown()) and rospy.Time.now() < t + rospy.Duration(10):
        c.set_vel(0.5,0,0)
    
    print("Velocity Setpoint 0 0.5 0")
    t = rospy.Time.now()
    while (not rospy.is_shutdown()) and rospy.Time.now() < t + rospy.Duration(10):
        c.set_vel(0,0.5,0)

    print("Landing")
    c.land()

def eight_vel():
    c = MavController()
    rospy.sleep(1)
    alt = 3.5
    print("Takeoff " + str(alt))
    c.takeoff(alt)
    rospy.sleep(5)

    t_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() - t_start < rospy.Duration(15):
        c.set_vel(0,0,0,avz = 0.1)
        rospy.loginfo("pose: %f, %f, %f, %f, %f, %f, %f",
            c.pose.position.x, c.pose.position.y, c.pose.position.z,
            c.pose.orientation.x, c.pose.orientation.y,
            c.pose.orientation.z, c.pose.orientation.w)

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
            c.set_vel(speed * multiplier * math.sin(t2), speed * math.cos(t2), 0)
            t = rospy.Time.now() - t_start

    for _ in range(2):
        circle_flying(speed, period, True)
        circle_flying(speed, period, False)

    print("Landing")
    c.land()

if __name__=="__main__":
    # simple_demo()
    # shape_eight()
    eight_vel()


