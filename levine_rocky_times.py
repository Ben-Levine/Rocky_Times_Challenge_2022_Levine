"""
testing offboard positon control with a simple takeoff script
"""
# Benjamin Levine
# SES 598: Autonomous Exploration Systems, SP22
# Dr. Jnaneshwar Das

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

class OffbPosCtl:
    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 1
    sim_ctr = 1

    des_pose = PoseStamped()
    isReadyToFly = False
    # location
    orientation = quaternion_from_euler(0, 0, 3.14 / 2 + 3.14 / 8)

    # Location per Gazebo:
    # PB: 60.208121, -12.502033, 18.775096
    # Probe: 40.666180, 3.675161, 11.148304
    # Rover: 12.6214, -65.7494, -3.502830
    locations = numpy.matrix([
                              [58, -11, 25, -orientation[0], -orientation[1], -orientation[2], -orientation[3]],
                              [60, -11, 25, -orientation[0], -orientation[1], -orientation[2], -orientation[3]],
                              [62, -11, 25, -orientation[0], -orientation[1], -orientation[2], -orientation[3]],
                              [62, -14, 25, -orientation[0], -orientation[1], -orientation[2], -orientation[3]],
                              [60, -14, 25, -orientation[0], -orientation[1], -orientation[2], -orientation[3]],
                              [58, -14, 25, -orientation[0], -orientation[1], -orientation[2], -orientation[3]],
                              [40.75, 3.75, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [40.68, 3.68, 11, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [20, -30, 15, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [12.75, -65.2, 0, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [12.75, -64.8, -3.5, orientation[0], orientation[1], orientation[2], orientation[3]],
                              ])

    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)
        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]

        # Comm for drones
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape

        self.landed = False

        while not rospy.is_shutdown():
            #print self.sim_ctr, shape[0], self.waypointIndex
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s" % e)

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException, e:
                    print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = shape[0]

                # Land drone
                land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
                land(altitude = self.des_pose.pose.position.z, latitude = self.des_pose.pose.position.x, longitude = self.des_pose.pose.position.y, min_pitch = 0, yaw = self.des_pose.pose.orientation.w)
                self.landed = True


            # Attach drone to probe
            if self.waypointIndex is 8:
                attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
                attach_srv.wait_for_service()
                link_probe = AttachRequest()
                link_probe.model_name_1 = "iris"
                link_probe.link_name_1 = "base_link"
                link_probe.model_name_2 = "sample_probe"
                link_probe.link_name_2 = "base_link"

                attach_srv.call(link_probe)

            # Drop probe in cart
            if self.waypointIndex is 10:
                detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
                detach_srv.wait_for_service()
                link_probe = AttachRequest()
                link_probe.model_name_1 = "iris"
                link_probe.link_name_1 = "base_link"
                link_probe.model_name_2 = "sample_probe"
                link_probe.link_name_2 = "base_link"

                detach_srv.call(link_probe)

            if self.isReadyToFly:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold and not self.landed:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14/2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        #print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            #print "readyToFly"


if __name__ == "__main__":
    OffbPosCtl()
