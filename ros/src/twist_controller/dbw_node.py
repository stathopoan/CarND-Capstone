#!/usr/bin/env python

import math
import threading
import time
import copy
import rospy
import numpy as np
import os
import sys

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd
from yaw_controller import YawController
from pid import PID
from twist_controller import GAS_DENSITY
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
from lowpass import SimpleLowPassFilter

this_file_dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(this_file_dir_path+'/../tools')
from utils import unpack_pose

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


def rad2deg(rad):
    return rad / math.pi * 180


def deg2rad(deg):
    return deg / 180. * math.pi


def cte_from_waypoints(car_x, car_y, car_yaw, waypoints):
    wp_transformed_x = []
    wp_transformed_y = []
    for wp in waypoints:
        wp_x = wp.pose.pose.position.x
        wp_y = wp.pose.pose.position.y
        shift_x = wp_x - car_x
        shift_y = wp_y - car_y
        x_rot = (shift_x * math.cos(-car_yaw) - shift_y * math.sin(-car_yaw))
        y_rot = (shift_x * math.sin(-car_yaw) + shift_y * math.cos(-car_yaw))
        wp_transformed_x.append(x_rot)
        wp_transformed_y.append(y_rot)
    coeffs = np.polyfit(wp_transformed_x, wp_transformed_y, 3)
    cte = np.polyval(coeffs, .0)
    return cte


def get_progressive_file_name(root, ext):
    i = 0
    while os.path.exists("{}{:04d}.{}".format(root, i, ext)):
        i += 1
    return "{}{:04d}.{}".format(root, i, ext)


class DBWNode(object):
    def __init__(self):
        cwd = os.getcwd()

        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600.  # m/s

        self.max_steer_angle = max_steer_angle

        self.current_linear_velocity = .0
        self.current_yaw_velocity = .0
        self.current_velocity_lock = threading.Lock()

        self.dbw_enabled = False
        self.dbw_enabled_lock = threading.Lock()

        self.last_twist_cb_time = None  # Only read/write this inside twist_cb(), it is not protected by locks!

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        '''
        The max deceleration torque the car is capable of, taken with positive sign.
        GAS_DENSITY is expressed in kg/US gallon, needs to be converted into kg/m**3
        '''
        self.max_decel_torque = abs(
            (vehicle_mass + fuel_capacity * GAS_DENSITY / .00378541) * decel_limit * wheel_radius)
        self.torque_deadband = (vehicle_mass + fuel_capacity * GAS_DENSITY / .00378541) * brake_deadband * wheel_radius

        self.pose_x, self.pose_y, self.pose_yaw = None, None, None
        self.pose_lock = threading.Lock()

        self.final_waypoints = []
        self.final_waypoints_lock = threading.Lock()

        self.twist = None
        self.twist_lock = threading.Lock()

        steering_filter_coeff = .8
        self.steering_filter = SimpleLowPassFilter(steering_filter_coeff)

        self.total_time = .0
        self.count = .0

        throttle_PID = .4, .015, 0.0  # Best so far

        path_to_dir = os.path.expanduser('~/.ros/chart_data')  # Replace ~ with path to user home directory
        f_name = get_progressive_file_name(path_to_dir, 'txt')
        self.data_out_file = open(f_name, 'w')
        assert self.data_out_file is not None
        rospy.loginfo('Writing performance data to file {}'.format(f_name))
        # Write headers for file
        self.data_out_file.write(
            'throttle PID={} - Filter on steering={}\n'.format(throttle_PID, steering_filter_coeff))
        self.data_out_file.write(
            'Iteration wanted_velocity throttle brake steer linear_v_error angular_v_error cte delta_t processing_time avg_proc_time\n')

        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=0,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

        # The throttle controller provides values in [-1, 1] range, to be scaled before actuation
        self.throttle_controller = PID(throttle_PID[0], throttle_PID[1], throttle_PID[2], mn=-1, mx=1)

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.DBW_enabled_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)

        self.spin()

    def spin(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            # Time-keeping
            current_time = time.time()
            if self.last_twist_cb_time is None:
                self.last_twist_cb_time = current_time
            else:
                self.total_time += current_time - self.last_twist_cb_time
                self.last_twist_cb_time = current_time
                self.count += 1.

                '''
                Time interval between two messages. This is the expected time. If I use instead the actual time elapsed
                since the previous call to the call-back, the variance in delta_t screws up the PID controllers.
                '''
                delta_t = .02

                '''
                If DBW is disabled, just return. You do not want the PID controllers to update the cumulative I error in this
                case.
                '''
                twist = self.get_twist()
                if self.get_dbw_enabled() and twist is not None:
                    if twist.linear.x < 0:
                        rospy.logdebug("Negative velocity in twist command: twist.linear.x={}".format(twist.linear.x))
                    if np.isclose(twist.linear.x, .0) and not np.isclose(twist.angular.z, .0):
                        rospy.logdebug("Null velocity {} with non null angular velocity {} in twist command".format(twist.linear.x, twist.angular.z))
                        wanted_velocity = self.speed_limit * .5
                    else:
                        wanted_velocity = abs(twist.linear.x)
                    wanted_angular_velocity = twist.angular.z
                    current_linear_v, current_angular_v = self.get_current_velocity()
                    steering = self.yaw_controller.get_steering(linear_velocity=wanted_velocity,
                                                                angular_velocity=wanted_angular_velocity,
                                                                current_velocity=current_linear_v)
                    steering = self.steering_filter.filt(steering)

                    linear_v_error = wanted_velocity - current_linear_v
                    throttle = self.throttle_controller.step(linear_v_error, delta_t)

                    if throttle > 0:
                        throttle_cmd = throttle
                        brake_cmd = .0
                    elif throttle < 0:
                        throttle_cmd = 0
                        brake_cmd = - 3 * self.max_decel_torque * throttle
                        if brake_cmd > self.max_decel_torque:
                            brake_cmd = self.max_decel_torque
                        if brake_cmd < self.torque_deadband:
                            brake_cmd = 0
                    else:
                        throttle_cmd = .0
                        brake_cmd = .0

                    if np.isclose(wanted_velocity, .0) and abs(linear_v_error) <= 2:
                        throttle_cmd = .0
                        brake_cmd = self.max_decel_torque / 3.

                    if throttle_cmd < 0 or throttle_cmd > 1:
                        rospy.logdebug("Throttle command out of range={}".format(throttle_cmd))
                    if brake_cmd < 0 or brake_cmd > self.max_decel_torque:
                        rospy.logdebug("Brake command out of range={}".format(brake_cmd))
                    if abs(steering) > self.max_steer_angle:
                        rospy.logdebug("Steering command out of range={}".format(steering))

                    self.publish(throttle=throttle_cmd, brake=brake_cmd, steer=steering)

                    '''
                    Processing below is kept after self.publish(), in order to minimise delay in propagating steering/throttle/brake
                    commands
                    '''

                    processing_time = time.time() - current_time

                    # Estimate cross-track error
                    path = self.get_final_waypoints(max_n_points=8)
                    pose_x, pose_y, pose_yaw = self.get_pose()
                    if len(path) > 0 and pose_x is not None:
                        cte = cte_from_waypoints(pose_x, pose_y, pose_yaw, path)
                    else:
                        cte = 0

                    # Evaluate more metrics to be reported
                    angular_vel_error = wanted_angular_velocity - current_angular_v
                    avg_processing_time = self.total_time / self.count

                    # Write to the log file used for off-line metrics charting and reporting
                    data_msg = '{} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}\n'
                    self.data_out_file.write(
                        data_msg.format(self.count, wanted_velocity, throttle_cmd, brake_cmd, steering, linear_v_error,
                                        angular_vel_error, cte, delta_t, processing_time, avg_processing_time))

                    """
                    # Write to ROS log files
                    log_msg = '#{} wanted_velocity={:.4f} throttle={:.4f} brake={:.4f} steer={:.4f} linear_v_error={:.4f} cte={:.4f} delta_t={:.4f} processing_time={:.4f} avg proc time={:.4f}'
                    rospy.logdebug(
                        log_msg.format(self.count, wanted_velocity, throttle_cmd, brake_cmd, steering, linear_v_error,
                                       cte, delta_t,
                                       processing_time, avg_processing_time))
                    """

            rate.sleep()

    def set_twist(self, twist):
        self.twist_lock.acquire()
        self.twist = twist
        self.twist_lock.release()

    def get_twist(self):
        self.twist_lock.acquire()
        twist = copy.deepcopy(self.twist)
        self.twist_lock.release()
        return twist

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

    def set_dbw_enabled(self, enable):
        self.dbw_enabled_lock.acquire()
        prev_value = self.dbw_enabled
        self.dbw_enabled = enable
        if prev_value == False and enable == True:
            self.throttle_controller.reset()
        self.dbw_enabled_lock.release()

    def get_dbw_enabled(self):
        self.dbw_enabled_lock.acquire()
        enabled = self.dbw_enabled
        self.dbw_enabled_lock.release()
        return enabled

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def final_waypoints_cb(self, msg):
        self.final_waypoints_lock.acquire()
        self.final_waypoints = msg.waypoints
        self.final_waypoints_lock.release()

    def get_final_waypoints(self, max_n_points):
        self.final_waypoints_lock.acquire()
        waypoints = copy.deepcopy(self.final_waypoints[0:max_n_points])
        self.final_waypoints_lock.release()
        return waypoints

    def get_pose(self):
        self.pose_lock.acquire()
        pose = self.pose_x, self.pose_y, self.pose_yaw
        self.pose_lock.release()
        return pose

    def pose_cb(self, msg):
        self.pose_lock.acquire()
        self.pose_x, self.pose_y, self.pose_yaw = unpack_pose(msg.pose)
        self.pose_lock.release()

    def twist_cb(self, msg):  # This is called at 30 Hz
        self.set_twist(msg.twist)

    def current_velocity_cb(self, msg):
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
        self.set_current_velocity(linear=linear, angular=angular)
        if linear > self.speed_limit:
            rospy.logdebug("Current speed {} exceeds speed limit {}".format(linear, self.speed_limit))

    def DBW_enabled_cb(self, msg):
        self.set_dbw_enabled(msg.data)


if __name__ == '__main__':
    DBWNode()

"""
TODO
====

+ test with video recording, ensure acceleration is robust
+ test with lower speed limits
+ test without GPU
+ check all TODOs
+ fix the way for TL detection to warm-up
! charting should show an empty track when at red lights
! handle stop at the end of the track
! anticipate all TLs by a few meters
! correctly set deceleration
! check/lower frequency of TL detection
! should it stop with a yellow light? Yes!

** tunare i filtri su throttle/brake/steering
- servono veramente le deep copy?
- controlla il time-stamp degli eventi in arrivo a twist_cb()
- considera di prendere il max torque da BrakCmd.TORQUE_MAX

"""
