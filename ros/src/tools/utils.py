import math
import tf

def unpack_pose(the_pose):
    """
    Extracts the three Cartesian coordinates from a geometry_msgs/Pose.

    :param pose: the given pose.
    :return: a tuple with the x, y and z coordinates extracted from the pose.
    """
    x = the_pose.position.x
    y = the_pose.position.y
    z = the_pose.position.z
    return x, y, z

def universal2car_ref(x, y, car_x, car_y, car_yaw):
    """
    Translates a 2D Cartesian coordinate from the universal reference system to the car reference system, where the car
    is at coordinates (0, 0) and with yaw 0 (the x-axis is oriented with the car).
    :param x: the x coordinate, in the universal reference system.
    :param y: the y coordinate, in the universal reference system.
    :param car_x: the car x coordinate.
    :param car_y:  the car y coordinate.
    :param car_yaw: the car yaw, in radians.
    :return: a pair of coordinates in the car reference system
    """
    shift_x = x - car_x
    shift_y = y - car_y
    x_res = (shift_x * math.cos(-car_yaw) - shift_y * math.sin(-car_yaw))
    y_res = (shift_x * math.sin(-car_yaw) + shift_y * math.cos(-car_yaw))
    return x_res, y_res


def car2universal_ref(x, y, car_x, car_y, car_yaw):
    """
    Translates a 2D Cartesian coordinate from the car reference system to the universal reference system.
    :param x: the x coordinate, in the car reference system.
    :param y: the y coordinate, in the car reference system.
    :param car_x: the car x coordinate.
    :param car_y: the car y coordinate.
    :param car_yaw: the car yaw.
    :return: a pair of coordinates in the universal reference system
    """
    unrotated_x = x * math.cos(car_yaw) - y * math.sin(car_yaw)
    unrotated_y = x * math.sin(car_yaw) + y * math.cos(car_yaw)
    x_res = unrotated_x + car_x;
    y_res = unrotated_y + car_y;
    return x_res, y_res


def poses_distance(pose1, pose2):
    """
    Computes the Euclidean distance between two poses.
    :param pose1: the first given pose.
    :param pose2: the second given pose.
    :return: the distance between poses.
    """
    x1, y1, z1 = unpack_pose(pose1)
    x2, y2, z2 = unpack_pose(pose2)
    distance = ((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)**.5
    return distance


def get_bearing_from_pose(my_pose, from_pose):
    """
    Computes the bearing of a second pose, as seen from a first pose.
    :param my_pose: the pose from which the bearing is observed.
    :param from_pose: the pose whose bearing is taken.
    :return: the computed bearing, in radians.
    """
    from_x, from_y, _ = unpack_pose(from_pose)
    bearing = get_bearing_from_xy(my_pose, from_x, from_y)
    return bearing


def get_bearing_from_xy(my_pose, from_x, from_y):
    my_x, my_y, _ = unpack_pose(my_pose)
    bearing = math.atan2(from_y-my_y, from_x-my_x)
    return bearing


def get_next_waypoint_idx(pose, waypoints, starting_idx):
    """
    Determines the index of the first waypoint in front of the car in a list of waypoints, starting from a given
    index. Note that the car is assumed to moved toward waypoints of increasing index (not allowed to go backward, or
    the other way around).
    :param pose: the car pose, inclusive of orientation.
    :param waypoints: the list of waypoints.
    :param starting_idx: starting position in parameter `waypoints` from which the index is searched.
    :return: the found index.
    """
    # Get the car yaw, x and y coordinates (in the universal reference system)
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    _, _, pose_yaw = tf.transformations.euler_from_quaternion(quaternion)
    pose_x, pose_y, _ = unpack_pose(pose)

    ''' Starting from the waypoint with index `starting_idx` in `waypoints`, find the first one with a positive
    x coordinate in the car reference system. That is the first waypoint in front of the car. '''
    wp_i = starting_idx
    direction = 1  # Currently always set to 1. Setting it to -1 would allow to search backward in the waypoints list.
    while True:
        # Fetch the waypoint coordinates, in the universal reference system.
        wp_x, wp_y, _ = unpack_pose(waypoints[wp_i].pose.pose)
        # Convert them to the car reference system
        wp_x_car_ref, _ = universal2car_ref(x=wp_x, y=wp_y, car_x=pose_x, car_y=pose_y, car_yaw=pose_yaw)
        # If the x coordinate is positive, then the waypoint is in front of the car, what we are looking for.
        if wp_x_car_ref >= 0:
            break
        # Otherwise check the next waypoint in the list; take care to wrap around when reaching the end of the list.
        wp_i = (wp_i + direction) % len(waypoints)
        assert wp_i != starting_idx  # If it gets in an infinite loop, then it is a bug!

    return wp_i


def distance(waypoints, wp1, wp2):
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    for i in range(wp1, wp2+1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist
