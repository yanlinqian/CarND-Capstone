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

import numpy as np

STATE_COUNT_THRESHOLD = 3


class SimplePose():
    '''Just work with x,y,z, theta'''

    def __init__(self, pose=PoseStamped()):
        self.pose = pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z

        euler = tf.transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w])
        self.theta = euler[2]

    def __repr__(self):
        return '<%.2f,%.2f,%.2f> (%.2f)' % (self.x, self.y, self.z, self.theta)


def distance(a, b):
    '''return distance between two simple poses'''
    return np.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def get_nearest_waypoint(pos, waypoints):
    '''returns the nearest waypoint index given a simplepos
    '''
    min_dist = 1e12
    min_index = 0
    for i, w in enumerate(waypoints):
        wpose = SimplePose(w.pose.pose)

        dist = distance(pos, wpose)
        heading = np.arctan2(pos.y - wpose.y, pos.x - wpose.x)
        dtheta = np.abs(heading - pos.theta)
        if dist < min_dist and (dtheta > np.pi / 4):
            min_dist = dist
            min_index = i
    return min_index

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

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

        self.light_position=self.config['stop_line_positions']
        self.light_wp=[]

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = SimplePose(msg.pose)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        self.light_wp=[]
        for position in self.light_position:
            wp_position=self.get_closest_waypoint_to_point(position[0],position[1],self.waypoints)
            self.light_wp.append(wp_position)

        self.sub2.unregister()

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.logerr('tl_detector, lights number: %d',len(self.lights))

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #rospy.logerr('tl_detector,light_wp:%d,state:%d'%(light_wp,state))
        # rospy.logerr('tl_detector,self.state:%d,count:%d'% (self.state,self.state_count))
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            #rospy.logerr('tl_detector,upcoming_red_light_pub.publish1:%d'% (light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #rospy.logerr('tl_detector,upcoming_red_light_pub.publish2:%d' % (self.last_wp))
        self.state_count += 1


    def get_closest_waypoint_to_point(self, pos_x, pos_y, waypoints):
        """Identifies the closest path waypoint to the given position
           https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """

        best_waypoint = None

        def distance2D(x1, y1, x2, y2):
            """Calculates dinstace from one point to another in 2D"""
            delta_x = x1 - x2
            delta_y = y1 - y2
            return math.sqrt(delta_x * delta_x + delta_y * delta_y)

        wps = waypoints.waypoints
        min_dist = distance2D(pos_x, pos_y,
                              wps[0].pose.pose.position.x,
                              wps[0].pose.pose.position.y)

        for i, point in enumerate(wps):
            dist = distance2D(pos_x, pos_y,
                              point.pose.pose.position.x,
                              point.pose.pose.position.y)
            if dist < min_dist:
                best_waypoint = i
                min_dist = dist

        return best_waypoint


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
           location and color.all
        Returns:
            int: index of waypoint closest to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light_positions = self.config['stop_line_positions']

        light = None
        light_wp = None

        if self.pose is not None and self.waypoints is not None:
            # Closest waypoint to current state
            car_position = self.get_closest_waypoint_to_point(self.pose.pose.position.x,self.pose.pose.position.y,self.waypoints)

            light_pos_wp = []
            for lpts in light_positions:
                l_pos = self.get_closest_waypoint_to_point(lpts[0], lpts[1], self.waypoints)
                light_pos_wp.append(l_pos)

            # get id of next light
            if car_position > max(light_pos_wp):
                light_wp = min(light_pos_wp)
            else:
                light_delta = light_pos_wp[:]
                light_delta[:] = [x - car_position for x in light_delta]
                light_wp = min(i for i in light_delta if i > 0) + car_position

            light_idx = light_pos_wp.index(light_wp)
            light = light_positions[light_idx]

        if light:
            state = self.get_light_state(light)

            if state == TrafficLight.RED:
                return light_wp, state
            else:
                return light_wp, -1
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('tl_detector, Could not start traffic node.')
