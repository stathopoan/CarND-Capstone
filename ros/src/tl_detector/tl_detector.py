#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import tensorflow
import sys
import os
import math

this_file_dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(this_file_dir_path+'/../tools')
from utils import get_next_waypoint_idx, unpack_pose, universal2car_ref, euler_distance


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.prev_wp_idx = 0

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # [x,y] coordinates of stopping lines before traffic lights
        self.stop_line_positions = self.config['stop_line_positions']
        # Indices in self.waypoints of the waypoints closest to the respective stopping lines, as reported in self.stop_line_positions
        self.stop_line_idxs = None  # Can be initialised only after receiving the list of waypoints, see waypoints_cb()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.stop_line_idxs = [np.argmin([euler_distance((wp.pose.pose.position.x, wp.pose.pose.position.y), stop_line_pos) for wp in self.waypoints]) for stop_line_pos in self.stop_line_positions]

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp_i, state = self.process_traffic_lights()
        # light_wp = self.lights[light_wp_i] if light_wp_i >= 0 else None

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp_i = light_wp_i if state == TrafficLight.RED else -1
            self.last_wp = light_wp_i
            self.upcoming_red_light_pub.publish(Int32(light_wp_i))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose_stamped):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        self.prev_wp_idx = get_next_waypoint_idx(pose_stamped.pose, self.waypoints, self.prev_wp_idx)
        return self.prev_wp_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose is not None:
            car_pose = self.pose.pose
            car_x, car_y, _ = unpack_pose(car_pose)
            #Find the closest visible traffic light (if one exists)
            min_distance = sys.float_info.max
            min_distance_i = -1
            for pos_i, pos in enumerate(stop_line_positions):
                dist = ((pos[0]-car_x)**2 + (pos[1]-car_y)**2) ** .5
                if dist < min_distance:
                    # bearing = get_bearing_from_xy(car_pose, pos[0], pos[1])
                    quaternion = (car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w)
                    _, _, car_yaw = tf.transformations.euler_from_quaternion(quaternion)
                    # rospy.logdebug("TL# {} at {} m with yaw = {} and bearing = {}".format(pos_i, dist, car_yaw/math.pi*180, bearing/math.pi*180))
                    tl_car_ref_x, _ = universal2car_ref(pos[0], pos[1], car_x, car_y, car_yaw)
                    if tl_car_ref_x >= .0:
                        min_distance = dist
                        min_distance_i = pos_i
            if min_distance_i >= 0 and min_distance < 50:  # TODO tune this min distance
                state = self.get_light_state(stop_line_positions[pos_i])  # TODO is this the right argument to pass?
                return self.stop_line_idxs[min_distance_i], state
            else:
                return -1, TrafficLight.UNKNOWN
        # self.waypoints = None
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
