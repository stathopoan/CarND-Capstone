#!/usr/bin/env python

import sys
import os
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import math
import threading
import time

this_file_dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(this_file_dir_path+'/../tools')
from utils import unpack_pose, distance, get_next_waypoint_idx


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


def plan_stop(wps, idx, max_decel, speed_limit):
    """
    Alter the speed of given waypoints to ensure the car comes to a full stop around the waypoints of index idx
    :param wps: the given waypoints
    :param idx: the index of the waypoint around which the car must stop
    :param speed: the initial speed of the car, in m/s
    :param max_decel: the maximum deceleration the car is capable of, a negative numer in m/s/s
    :param speed_limit: the speed limit not be exceeded, in m/s
    """

    if idx < 0:
        return []

    wps = wps[0: idx+1]

    wps[idx].twist.twist.linear.x = 0
    current_speed = 0
    current_i = idx-1
    while current_i >= 0 and (current_i == 0 or current_speed < wps[current_i-1].twist.twist.linear.x):
        dist = distance(wps, current_i, current_i+1)
        current_speed = (current_speed**2 - 2*max_decel*dist)**.5
        if current_i >= 1:
            current_speed = min(current_speed, wps[current_i-1].twist.twist.linear.x)
        else:
            current_speed = min(current_speed, speed_limit)
        wps[current_i].twist.twist.linear.x = current_speed
        current_i -= 1

    return wps


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        ''' Note that subscription to /current_pose is done inside self.waypoints_cb(). No point in receiving pose
        updates before having received the track waypoints. '''
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # The configured speed limit
        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600.  # m/s

        # The speed limit that will be enforced, lower than the configured speed limit, in order to have a safety margin
        self.enforced_speed_limit = min(self.speed_limit * .9, self.speed_limit - 1.)
        if self.enforced_speed_limit <= 0:
            self.enforced_speed_limit = self.speed_limit * .9

        # The waypoints describing the track and the wanted cruise speed (traffic lights aside)
        self.waypoints = None

        ''' Every time a /current_pose message is received, method pose_cb() determines the first track waypoint
        in front of the car. To do so, it scans the list of waypoints in self.waypoints. Once it finds the
        wanted waypoint, it saves its index in self.prev_wp_idx (here below).
        Next time a /current_pose event is received, method pose_cb() will start searching self.waypoints from
        the position saved in self.prev_we_idx; this is more efficient than looking every time starting from
        the beginning of the list. '''
        self.prev_wp_idx = 0

        self.received_pose_count = 0  # Counts how many car pose updates (/current_pose messages) have been received
        self.previous_pose_cb_time = None  # Time of when the last pose (/current_pose messages) has been received
        self.total_time = .0  # Total time spent in executing pose_cb(), for performance monitoring
        # Limit the frequency of processing of /current_pose, at most once every these many seconds
        self.min_update_int = .2

        self.current_linear_velocity = .0
        self.current_yaw_velocity = .0
        self.current_velocity_lock = threading.Lock()

        rospy.spin()

    def pose_cb(self, msg):  # This is being called at 50 Hz
        """
        Processes pose update messages. The current implementation takes a list of LOOKAHEAD_WPS waypoints,
        out of the track waypoints, that are in front of the car, set them at a constant speed, and publishes them for
        the waypoint_follower to follow. The car will try to follow those waypoints at constant speed (ignoring traffic
        lights). TODO update to keep traffic ligths into consideration.
        :param msg: the received message.
        """

        # Update time and count information about received poses
        now = time.time()
        delta_t = 0 if self.previous_pose_cb_time is None else now-self.previous_pose_cb_time
        if self.previous_pose_cb_time is None:
            self.previous_pose_cb_time = now
        if delta_t < self.min_update_int:
            return
        self.previous_pose_cb_time = now
        self.total_time += delta_t
        self.received_pose_count += 1 # TODO protect with mutex!

        rospy.logdebug('Processing pose #{}; delta_t from previous processing is {}s with average {}s'.format(self.received_pose_count,
                                                                                                              delta_t,
                                                                                                              self.total_time/self.received_pose_count))

        # Determine the index in `self.waypoints` of the first waypoint in front of the car
        pose_i = get_next_waypoint_idx(msg.pose, self.waypoints, self.prev_wp_idx)
        # spy.logdebug('Next wp index is {}'.format(pose_i))
        self.prev_wp_idx = pose_i

        # Collect LOOKAHEAD_WPS waypoints starting from that index, and publish them.
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        for count in xrange(LOOKAHEAD_WPS):
            i = (pose_i+count) % len(self.waypoints)
            wp = self.waypoints[i]
            # Cap the linear velocity at self.enforced_speed_limit
            wp.twist.twist.linear.x = min(wp.twist.twist.linear.x, self.enforced_speed_limit)
            lane.waypoints.append(wp)

        if False:  # Set to True if you want the car to stop at the second traffic light (waypoint #750), for testing.
            if pose_i+LOOKAHEAD_WPS > 750:
                current_vel, _ = self.get_current_velocity()
                lane.waypoints = plan_stop(lane.waypoints, 750-pose_i, -2, self.enforced_speed_limit)
            if pose_i >= 750:
                lane.waypoints = []

        self.final_waypoints_pub.publish(lane)
        total_time = time.time() - now
        rospy.logdebug('Time spent in pose_cb: {}s'.format(total_time))

    def waypoints_cb(self, waypoints):
        """
        Receives the list of waypoints making the track and store it in self.waypoints
        :param waypoints: the received message
        """
        # This callback should be called only once, with the list of waypoints not yet initialised.
        assert self.waypoints is None

        for wp in waypoints.waypoints:
            wp.twist.twist.linear.x = 9.

        self.waypoints = waypoints.waypoints # No need to guarantee mutual exclusion in accessing this data member

        # Now that the waypoints describing the track have been received, it is time to subscribe to pose updates.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):  # TODO remove code duplication with DBW node
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
        self.set_current_velocity(linear=linear, angular=angular)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def set_current_velocity(self, linear, angular):
        self.current_velocity_lock.acquire()
        self.current_linear_velocity = linear
        self.current_yaw_velocity = angular
        self.current_velocity_lock.release()

    def get_current_velocity(self):
        self.current_velocity_lock.acquire()
        linear = self.current_linear_velocity
        angular = self.current_yaw_velocity
        self.current_velocity_lock.release()
        return linear, angular


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
