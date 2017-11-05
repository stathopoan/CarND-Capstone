#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import sys
import tf
import math
import threading
import time

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


def universal2car_ref(x, y, car_x, car_y, car_yaw):
    shift_x = x - car_x
    shift_y = y - car_y
    x_res = (shift_x * math.cos(-car_yaw) - shift_y * math.sin(-car_yaw))
    y_res = (shift_x * math.sin(-car_yaw) + shift_y * math.cos(-car_yaw))
    return x_res, y_res


def car2universal_ref(x, y, car_x, car_y, car_yaw):
    unrotated_x = x * math.cos(car_yaw) - y * math.sin(car_yaw)
    unrotated_y = x * math.sin(car_yaw) + y * math.cos(car_yaw)
    x_res = unrotated_x + car_x;
    y_res = unrotated_y + car_y;
    return x_res, y_res


def unpack_pose(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    return x, y, z


def poses_distance(pose1, pose2):
    x1, y1, z1 = unpack_pose(pose1)
    x2, y2, z2 = unpack_pose(pose2)
    distance = ((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)**.5
    return distance


def get_bearing_from_pose(my_pose, from_pose):
    my_x, my_y, _ = unpack_pose(my_pose)
    from_x, from_y, _ = unpack_pose(from_pose)
    bearing = math.atan2(from_y-my_y, from_x-my_x)
    return bearing


def get_next_waypoint_idx(pose, waypoints, starting_idx):
    # Get the car yaw, x and y coordinates
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, pose_yaw = tf.transformations.euler_from_quaternion(quaternion)
    pose_x, pose_y, _ = unpack_pose(pose)
    direction = 1
    wp_i = starting_idx

    ''' Starting from the waypoint with index starting_idx in waypoints, find the first one with a positive
    x coordinate in the car reference system '''
    while True:
        wp_x, wp_y, _ = unpack_pose(waypoints[wp_i].pose.pose)
        wp_x_car_ref, _ = universal2car_ref(x=wp_x, y=wp_y, car_x=pose_x, car_y=pose_y, car_yaw=pose_yaw)
        if wp_x_car_ref >= 0:
            break
        wp_i = (wp_i + direction) % len(waypoints)
        assert wp_i != starting_idx  # Should not get into an infinite loop

    return wp_i


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        # rospy.logdebug('Inside WaypointUpdater.__init__()')
        # rospy.logdebug('Running Python version '+sys.version)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        # The index in self.waypoints[] of the last waypoint found to be the closest in front of the car
        self.prev_wp_idx = 0
        self.received_pose_count = 0
        # self.lock = threading.Lock()
        self.previous_pose_cb_time = None

        self.total_time = .0

        rospy.spin()

    def pose_cb(self, msg):  # This is being called at 50 Hz
        now = time.time()
        delta_t = 0 if self.previous_pose_cb_time is None else now-self.previous_pose_cb_time
        self.total_time += delta_t
        self.previous_pose_cb_time = now
        self.received_pose_count += 1 # TODO protect with mutex!

        rospy.logdebug('Processing pose #{}'.format(self.received_pose_count))
        rospy.logdebug('delta_t from previous processing is {} with average {}'.format(delta_t, self.total_time/self.received_pose_count))
        pose_i = get_next_waypoint_idx(msg.pose, self.waypoints, self.prev_wp_idx)
        self.prev_wp_idx = pose_i
        rospy.logdebug('Next waypoint is #{}'.format(pose_i))
        direction = 1
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        for count in xrange(LOOKAHEAD_WPS):
            i = (pose_i+count*direction) % len(self.waypoints)
            wp =self.waypoints[i]
            wp.twist.twist.linear.x = 9.
            lane.waypoints.append(wp)
        self.final_waypoints_pub.publish(lane)
        # total_time = time.time() - start_time
        # rospy.logdebug('Time spent in pose_cb: {}'.format(total_time))


    def waypoints_cb(self, waypoints):
        assert self.waypoints is None
        self.waypoints= waypoints.waypoints
        rospy.logdebug('Received {} waypoints:'.format(len(waypoints.waypoints)))
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
