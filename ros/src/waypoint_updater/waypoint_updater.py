#!/usr/bin/env python

import sys
import os
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf
import math
import threading
import time

this_file_dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(this_file_dir_path+'/../tools')
from utils import unpack_pose, distance, get_next_waypoint_idx, poses_distance


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


def plan_stop(wps, idx, decel, speed_limit):
    """
    Alter the speed of given waypoints to ensure the car comes to a full stop around the waypoints of index idx
    :param wps: the given waypoints
    :param idx: the index of the waypoint within wps where the car must stop
    :param speed: the initial speed of the car, in m/s
    :param decel: the wanted deceleration, a negative numer in m/s/s
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
        current_speed = (current_speed**2 - 2*decel*dist)**.5
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

        self.decel_limit = rospy.get_param('/dbw_node/decel_limit', -5)
        self.accel_limit = rospy.get_param('/deb_node/accel_limit', 1.)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

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

        self.tl = -1
        self.tl_lock = threading.Lock()

        rospy.spin()

    def set_tl(self, value):
        self.tl_lock.acquire()
        self.tl = value
        self.tl_lock.release()

    def get_tl(self):
        return self.tl

    def pose_cb(self, msg):  # This is being called at 50 Hz
        """
        Processes pose update messages.
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
        # Store the found value; search will start from it at the next iteration, for efficiency
        self.prev_wp_idx = max(pose_i, self.prev_wp_idx)   # Cannot go backward!

        # Collect LOOKAHEAD_WPS waypoints starting from that index
        # TODO as optimization, this part could be skipped if stopping by a red light, as lane.waypoints will be set to [] below
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        for count in xrange(LOOKAHEAD_WPS):
            i = (pose_i+count) % len(self.waypoints)
            wp = self.waypoints[i]
            # Cap the linear velocity at self.enforced_speed_limit
            wp.twist.twist.linear.x = min(wp.twist.twist.linear.x, self.enforced_speed_limit)
            lane.waypoints.append(wp)

        # Handle traffic lights
        tl_wp_i = self.get_tl()
        rospy.logdebug("Getting tl_wp_i={}".format(tl_wp_i))
        if tl_wp_i >=0:  # If there is a red traffic light in front of the car...
            # ... target to stop a couple waypoints before it, as a margin of safety
            # tl_wp_i = (tl_wp_i - 1) % len(self.waypoints)
            # If already at (or past) the stop waypoint, make sure the car stops and doesn't move
            if pose_i >= tl_wp_i:
                lane.waypoints = []
            # If the waypoint where to stop is within LOOKAHEAD_WPS from the current closest waypoint...
            elif pose_i+LOOKAHEAD_WPS > tl_wp_i:
                # ... then plan to stop
                lane.waypoints = plan_stop(lane.waypoints, tl_wp_i-pose_i, -2, self.enforced_speed_limit)  # TODO set the deceleration properly

        '''
        else:  # If there is no red traffic light in front of the car, make sure lane.waypoints 
            prev_velocity = lane.waypoints[-1].twist.twist.linear.x if len(lane.waypoints) > 0 else .0
            while len(lane.waypoints) < LOOKAHEAD_WPS:
                i = (pose_i + len(lane.waypoints) + 1) % len(self.waypoints)
                wp = self.waypoints[i]
                dist = distance(self.waypoints, i, (i-1) % len(self.waypoints))
                wp.twist.twist.linear.x = (prev_velocity**2 + 2*self.accel_limit*dist)**.5
                wp.twist.twist.linear.x = min(wp.twist.twist.linear.x, self.enforced_speed_limit, self.waypoints[i].twist.twist.linear.x)
                lane.waypoints.append(wp)
            
        '''

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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

    def traffic_cb(self, msg):
        # DONE: Callback for /traffic_waypoint message. Implement
        self.set_tl(msg.data)

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
