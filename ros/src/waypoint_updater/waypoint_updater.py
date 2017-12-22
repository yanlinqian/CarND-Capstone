#!/usr/bin/env python
'''
TASK 1: update base_waypoints to final_waypoints
'''
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math

import tf
import numpy as np
from std_msgs.msg import Int32,Bool




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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
RATE = 50 # time (in miniseconds) for updater's sleeping
REFERENCE_VELOCITY=4.47 # 4.47 m/s = ~ 10mph
BRAKE_VELOCITY=0
SLOW_VELOCITY=5

STOP_LINE_OFFSET=5


CAR_STATE_DRVING=1
CAR_STATE_STOPING=0

REQ_DEACC=2 #required deaccletion
MAX_DEACC=10


def get_closest_waypoint(pose_x, pose_y, theta, waypoints):
    """
        Get the closest waypoint for a given position
    :param pose_x:
    :param pose_y:
    :param theta:
    :param waypoints:
    :return:
    """
    # initial variables
    closest_distance = 100000.0
    closest_point = 0

    for i in range(len(waypoints)):
        # extract waypoint x,y
        wp_x = waypoints[i].pose.pose.position.x
        wp_y = waypoints[i].pose.pose.position.y
        # compute distance from car x,y
        distance = math.sqrt((wp_x - pose_x) ** 2 + (wp_y - pose_y) ** 2)
        # is this point closer than others found so far
        if distance < closest_distance:
            closest_distance = distance
            closest_point = i

    x = waypoints[closest_point].pose.pose.position.x
    y = waypoints[closest_point].pose.pose.position.y
    heading = np.arctan2((y - pose_y), (x - pose_x))
    angle = np.abs(theta - heading)
    if angle > np.pi / 4.:
        closest_point += 1
        if closest_point >= len(waypoints):
            closest_point = 0

    return closest_point

def get_stopping_distance(velocity,deacc):
    return velocity*velocity/deacc

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        #keep subcriber-pulisher logic clean, don't add subcribing to new topics
        #subcribing
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub=rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_cb)
        rospy.Subscriber('/current_velocity',TwistStamped,self.velocity_cb)

        #publishing
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_waypoints=None
        self.current_pos=None
        self.current_velocity=None
        self.trafficlight=-1
        self.dbw_enabled=False
        self.car_state=None
        self.pose_x=-1
        self.pose_y=-1
        self.theta=0

        self.loop()
        #rospy.spin()
    def loop(self):
        rate=rospy.Rate(RATE/5)
        while not rospy.is_shutdown():
            if self.current_waypoints is None or self.dbw_enabled is False\
    :
                #rospy.logerr('waiting')
                rate.sleep()
                continue
            if self.pose_x>-1 and self.pose_y>-1:
                id_cloest=get_closest_waypoint(self.pose_x,self.pose_y,self.theta,self.current_waypoints)


            start_velocity=self.get_waypoint_velocity(self.current_waypoints[id_cloest])
            #rospy.logerr('start_velocity:%d',start_velocity)
            #self.trafficlight = -1
            if self.trafficlight is None:
                self.trafficlight=-1

            if self.trafficlight == -1:
                self.car_state=CAR_STATE_DRVING
            else:#with trafficlight
                rospy.logerr('waypoint_updator,trafficlight:%d'%self.trafficlight)
                distance_to_light=self.distance(self.current_waypoints,id_cloest,self.trafficlight)
                req_distance=get_stopping_distance(self.current_velocity,REQ_DEACC)
                min_distance=get_stopping_distance(self.current_velocity,MAX_DEACC)
                if distance_to_light-STOP_LINE_OFFSET<req_distance:#no enough distance to brake comfortably
                    self.car_state=CAR_STATE_STOPING
                    rospy.logerr('waypoint_updator,not enough for req_distance:%d'%req_distance)
                else: #there is enough space to drive and then brake
                    #still some time to drive
                    self.car_state=CAR_STATE_DRVING
                    rospy.logerr('enough for req_distance:%d'%req_distance)
            rospy.logerr('waypoint_updator, car_state:%d'%self.car_state)

            #get velocity for the aheading waypoints
            if self.car_state==CAR_STATE_DRVING:
                #set the aheading wappoints as the reference speed, we follow them
                for i in range(id_cloest,id_cloest+LOOKAHEAD_WPS):
                    if i<len(self.current_waypoints):
                        self.set_waypoint_velocity(self.current_waypoints,i,REFERENCE_VELOCITY)
            else:
                #slow down gradually to the waypoint just before the red light
                for i in range(id_cloest,self.trafficlight+1):
                    if(distance_to_light-STOP_LINE_OFFSET)>0:
                        distance_to_i=self.distance(self.current_waypoints,id_cloest,i)
                        velocity_waypoint_i=start_velocity-start_velocity*(distance_to_i)/(distance_to_light-STOP_LINE_OFFSET)
                    else:
                        velocity_waypoint_i=-10
                    self.set_waypoint_velocity(self.current_waypoints, i, velocity_waypoint_i)


            waypoints_ahead=[]
            for i in range(LOOKAHEAD_WPS):
                if id_cloest+i<len(self.current_waypoints):
                    waypoints_ahead.append(self.current_waypoints[id_cloest+i])
            lane_ahead=Lane()
            lane_ahead.waypoints=waypoints_ahead
            lane_ahead.header.stamp=rospy.Time(0)

            self.final_waypoints_pub.publish(lane_ahead)
            rate.sleep()

        pass

    def pose_cb(self, msg):
        # Implement

        # msg will be a geometry_msgs/PoseStamped message
        # extract the current car x, y
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        orientation = msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
        self.theta = euler[2]

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.current_waypoints=waypoints.waypoints
        #the publiser for /base_waypoints publishes only once
        self.base_waypoints_sub.unregister()
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.trafficlight=msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later

        pass

    def dbw_cb(self,msg):
        self.dbw_enabled=msg.data

    def velocity_cb(self,msg):
        self.current_velocity=msg.twist.linear.x

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
