#!/usr/bin/env python
'''
TASK 1: update base_waypoints to final_waypoints
'''
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

import tf
import numpy as np
from std_msgs.msg import Int32




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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
RATE = 50 # time (in miniseconds) for updater's sleeping
REFERENCE_VELOCITY=4.457 # 4.5 m/s
BRAKE_VELOCITY=0
WPNUM_CONSIDER_BRAKE= 50# consider braking when we have number of waypoint gaps to traffic light


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
    '''returns the nearest waypoint index
       pos a simple pos
    '''
    min_dist = 1e12
    min_index = 0
    for i, w in enumerate(waypoints):
        wpose = SimplePose(w.pose.pose)

        dist = distance(pos, wpose)
        heading = np.arctan2(pos.y - wpose.y, pos.x - wpose.x)
        dtheta = np.abs(heading - pos.theta)
        if dist < min_dist and (dtheta <= np.pi / 4):
            min_dist = dist
            min_index = i
    return min_index

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        #keep subcriber-pulisher logic clean, don't add subcribing to new topics
        #subcribing
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub=rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)

        #publishing
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_waypoints=None
        self.current_pos=None
        self.trafficlight=None

        self.loop()
        #rospy.spin()
    def loop(self):
        rate=rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if self.current_pos is None or self.current_waypoints is None:
                #rospy.logerr('waiting')
                rate.sleep()
                continue

            id_cloest=get_nearest_waypoint(self.current_pos,self.current_waypoints)
            if self.trafficlight is None:
                self.trafficlight=-1
            #rospy.logerr('next waypoint: %i, next light: %d '%
            #             (id_cloest,self.trafficlight))

            new_waypoints=Lane()
            farest_waypoint=id_cloest+LOOKAHEAD_WPS
            current_v=REFERENCE_VELOCITY
            new_waypoints.waypoints=self.current_waypoints[id_cloest:farest_waypoint]

            if self.trafficlight>id_cloest and self.trafficlight-id_cloest<WPNUM_CONSIDER_BRAKE:
                #rospy.logerr('brake')
                current_v=BRAKE_VELOCITY

            for k,w in enumerate(new_waypoints.waypoints):
                w.twist.twist.linear.x=current_v

            self.final_waypoints_pub.publish(new_waypoints)
            rate.sleep()

        pass

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pos=SimplePose(msg.pose)

        pass

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
