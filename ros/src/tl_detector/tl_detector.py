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
import math

STATE_COUNT_THRESHOLD = 3
ENABLE_CLASSIFIER = False
LINE_OF_SIGHT = 100.0 # m

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
	self.lightWaypointsIdxs = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

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

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):	
        self.waypoints = waypoints.waypoints
	# only get it once - reduce resource consumption
        self.sub2.unregister()
	# Convert light coordinates to the nearest waypoint - Runs only once
	self.build_light_waypoints()

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
        light_wp, state = self.process_traffic_lights()
	#rospy.logdebug('light_wp: {} state: {}'.format(light_wp,state))

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp	    
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:           
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose): 
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO Perhaps it can be optimized to search in a specified window and not the entire list
        index = -1 
        if self.waypoints is None:
            return index
	
	min_distance = 1000000;
	for i in range(len(self.waypoints)):	   
	   distance = self.distance(pose,self.waypoints[i].pose.pose)
	   if distance < min_distance:
	      min_distance = distance
	      index = i
	   
        return index

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
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None # Here light is the actual waypoint that is closest to the next traffic light position

        #stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
	    if car_position >= 0:
	       light, light_wp_idx = self.get_closest_light_waypoint(car_position)
	       #rospy.logdebug('light: {}'.format(light))
	       #rospy.logdebug('light_ind: {}'.format(light_wp))
	
	    

        # find the closest visible traffic light (if one exists)

        if light:
	    # Initialize state 
	    state = TrafficLight.UNKNOWN
	    if ENABLE_CLASSIFIER:
	       #TODO
               state = self.get_light_state(light)
               
	    else:
	       # Use info from /vehicle/traffic_lights
	       # Iterate over all trafic lights
	       for l in self.lights:
	           # Compute the distance between light waypoint and light position given by topic
                   if self.distance(l.pose.pose,light.pose.pose) < 40.:
		      # If this the light we are looking for take its state
		      state = l.state
		      break
	    return light_wp_idx, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN


    

    def build_light_waypoints(self):
	""" 
	Find the closest waypoint to light position. Create a list of these mappings and keep it. 
	The list will have length equal to the number of traffic lights.
	"""
	assert self.waypoints is not None

	# List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
	# The distance formula / Ignore z in waypoints coord (Is this right?)	
	dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)	

	for light_idx in range(len(stop_line_positions)):
	   min_distance = 1000000
	   index=0
	   for wayp_idx in range(len(self.waypoints)):
	      distance = dl(self.waypoints[wayp_idx].pose.pose.position, stop_line_positions[light_idx])
	      if distance < min_distance:
	          index = wayp_idx
	          min_distance = distance
	   
	   self.lightWaypointsIdxs.append(index)
	
	'''for i in range(len(self.lightWaypoints)):
	   rospy.logdebug('light: {}'.format(stop_line_positions[i]))	      
	   rospy.logdebug('closest waypoint: {}'.format(self.waypoints[self.lightWaypoints[i]]))
	''' 
	return

    def get_closest_light_waypoint(self,car_position):
	"""
	Search in the self.lightWaypoints list where it contains all waypoint ids that represent traffic lights
	and find the one closest to our current position which is represented as a waypoint id. Note: The lookup 
	wrap around approach so if no waypoint matches our criteria

	:param 	car_position: The index of the nearest waypoint closest to the actual car pose
	
	:returns:
	       Waypoint: The nearest waypoint to the actual light position
	       int: The index of the corresponding waypoint in self.waypoints list
	"""
	
	assert( len(self.lightWaypointsIdxs) > 0 )
	
	light = None
	next_light_wp_ind = None
        
        for ind in range(len(self.lightWaypointsIdxs)):
           if (car_position<self.lightWaypointsIdxs[ind]):
              next_light_wp_ind = self.lightWaypointsIdxs[ind]
              break
        # If our position has passed all traffic lights in the first lap the next traffic light will be the first one in the new lap
        if next_light_wp_ind is None:
           next_light_wp_ind = self.lightWaypointsIdxs[0]
	
	# Compute distance between two waypoints. This is not accurate since it does not concatenate distances along the arc between points
	# but it is faster and provides an average estimation of the distance between two waypoints
	distance = self.distance_XY(self.waypoints[car_position].pose.pose, self.waypoints[next_light_wp_ind].pose.pose)
	
	# If distance within line of sight
	if distance <= LINE_OF_SIGHT:
	    light = self.waypoints[next_light_wp_ind]
	    return light,next_light_wp_ind
	
        return None,-1


    def distance_concat(self, waypoints, wp1, wp2):
	"""
	Computes the distance between two waypoints in a list along the piecewise linear arc connecting all waypoints between the two.
	"""
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_XY(self, pos1, pos2):
	"""
	Find the eucelidian distance between two waypoints in world coordinates ignoring z coordinate
	"""
        return math.sqrt((pos1.position.x - pos2.position.x)**2 + (pos1.position.y - pos2.position.y)**2)


    def distance(self, pos1, pos2):
	"""
	Find the eucelidian distance between two waypoints in world coordinates.

	"""
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)        
        dist = dl(pos1.position, pos2.position)
        return dist
	

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
