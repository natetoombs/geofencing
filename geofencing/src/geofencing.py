#!/usr/bin/env python3

# Nathan Toombs

import numpy as np
import rospy
import pyproj

from nav_msgs.msg import Odometry
from roscopter_msgs.msg import Failsafe
from rosflight_msgs.msg import GNSS
from geofencing_msgs.srv import AddGeoPoint, ClearGeofence

# TODO: GPS data come in ECEF, not LLA

class Geofencing():
    def __init__(self):

        # Get points
        try:
            self.list = rospy.get_param('~geofence_points')
        except KeyError:
            rospy.logfatal('[geofencing] Geofence points not set')
            rospy.signal_shutdown('[geofencing] Parameters not set')

        self.num_points = len(self.list)

        self.failsafe_msg = Failsafe()
        self.failsafe_msg.failsafe = False
        self.failsafe_msg.mode = Failsafe.RETURN_TO_BASE

        self.land_delay = rospy.get_param('~land_delay')
        self.time = 0
        self.time_flag = False
        self.print_flag1 = False
        self.print_flag2 = False

        self.converted = True
        if rospy.get_param('~gps_points',False): # TODO: Do we need to make sure the odometryCallback doesn't run before the list is converted?
            # Setup gps subscriber
            self.converted = False # Bool to determine if the list has been converted
            self.gps_sub_ = rospy.Subscriber('gps/data', GNSS, self.gpsCallback, queue_size=5)

        # Setup subscribers and publisher
        self.xhat_sub_ = rospy.Subscriber('odom', Odometry, self.odometryCallback, queue_size=5)
        self.failsafe_pub_ = rospy.Publisher('failsafe', Failsafe, queue_size=10)

        # Setup services
        self.add_point_service = rospy.Service('add_geofence_point', AddGeoPoint, self.addPointCallback)
        self.clear_geofence_service = rospy.Service('clear_geofence', ClearGeofence, self.clearGeofenceCallback)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def odometryCallback(self, msg):
        # Only deals with the NE coordinates of the UAV. Coordinates are relative to the origin. 
        # If the UAV exits the geofenced area, it is given the command to return to base, the 
        # failsafe message is set to true, and a timer starts. If the timer reaches a parameter 
        # `land_delay`, the commanded behavior changes to just land in place. As soon as the UAV 
        # returns to the geofenced area, the failsafe message is set to false.
        if self.converted == False:
            return
        
        self.n = msg.pose.pose.position.x
        self.e = msg.pose.pose.position.y

        point = [self.n, self.e]

        if self.num_points < 2:
            return
        if not self.is_in_bounds(point):
            self.failsafe_msg.failsafe = True
            if self.time_flag == False:
                self.time = rospy.get_time()
            self.time_flag = True
            if rospy.get_time() - self.time < self.land_delay:
                if self.print_flag1 == False:
                    rospy.loginfo("[geofencing] Out of Bounds; Returning to Base")
                    self.print_flag1 = True
                self.failsafe_msg.mode = Failsafe.RETURN_TO_BASE
            else:
                if self.print_flag2 == False:
                    rospy.loginfo("[geofencing] Out of Bounds; Landing")
                    self.print_flag2 = True
                self.failsafe_msg.mode = Failsafe.LAND
        else:
            self.failsafe_msg.failsafe = False
        self.failsafe_pub_.publish(self.failsafe_msg)

    def is_in_bounds(self, point):
        # Checks to see if the point is in bounds, within a polygon defined by
        # the points in order around the perimeter. Algorithm used from Darel 
        # Rex Finley (http://alienryderflex.com/polygon/)
        i = 0
        j = self.num_points - 1
        point_is_in = False
        for i in range(self.num_points):
            if (self.list[i][1] < point[1] and self.list[j][1] >= point[1] or self.list[j][1] < point[1] and self.list[i][1] >= point[1]) \
            and (self.list[i][0] <= point[0] or self.list[j][0] <= point[0]):
                if (self.list[i][0] + (point[1] - self.list[i][1])/(self.list[j][1] - self.list[i][1])*(self.list[j][0] - self.list[i][0]) < point[0]):
                    point_is_in = not point_is_in
            j = i
        return(point_is_in)

    def gpsCallback(self, msg):
        # Returns the first received GPS coordinate (LAT LON) and unregisters
        # the subscriber
        # TODO: Make sure the first message is good?
        self.origin = self.ecef2lla(msg.position[0],msg.position[1],msg.position[2])
        print("Origin: ", self.origin) # TODO: Remove
        self.convert_list()
        self.gps_sub_.unregister()

    def convert_list(self):
        # Uses the Haversine method to compute the distance between each point
        # in the list and the origin, putting the list in NE coordinates.
        print(self.list) # TODO: Remove
        for i in range(self.num_points):
            lat = self.list[i][0]
            lon = self.list[i][1]
            print(lat, self.origin[0])
            mag_n_pos = self.get_haversine_distance([self.origin[0],lon])
            print("Mag_n_pos: ", mag_n_pos)
            n_pos = mag_n_pos * np.sign(lat-self.origin[0])
            mag_e_pos = self.get_haversine_distance([lat,self.origin[1]])
            e_pos = mag_e_pos * np.sign(lon-self.origin[1])
            self.list[i] = [n_pos,e_pos]
        self.converted = True
        print("converted: ", self.list) # TODO: Remove
        return


    def get_haversine_distance(self, point):
        # Calculates the Haversine distances between two coordinates, in meters.
        lat1,lon1 = self.origin[0:2]
        print("Lat1,lon1: ", lat1, lon1)
        lat2,lon2 = point[0:2]
        print("Lat2,lon2: ", lat2, lon2)
        
        R = 6371000 # Radius of Earth, meters

        phi_1 = np.deg2rad(lat1)
        phi_2 = np.deg2rad(lat2)

        delta_phi = np.deg2rad(lat2 - lat1)
        delta_lambda = np.deg2rad(lon2 - lon1)

        a = np.sin(delta_phi/2.0)**2 + np.cos(phi_1)*np.cos(phi_2)*\
            np.sin(delta_lambda/2.0)**2
        c = 2.0*np.arctan2(np.sqrt(a), np.sqrt(1-a))
        print(c)

        distance = R * c # meters

        return distance

    def ecef2lla(self, x, y, z):
        ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
        lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        lon, lat, alt = pyproj.transform(ecef, lla, x, y, z, radians=False)
        return [lat, lon, alt]

    def addPointCallback(self, req):
        new_waypoint = [req.x, req.y]
        self.list.append(new_waypoint)
        self.num_points = len(self.list)
        rospy.loginfo("[geofencing] Added New Geofence Point")
        return True

    def clearGeofenceCallback(self, req):
        self.list = []
        self.num_points = 0
        rospy.loginfo("[geofencing] Geofence Cleared")
        return True

if __name__ == '__main__':
    rospy.init_node('geofencing', anonymous=True)
    try:
        geofencing = Geofencing()
    except:
        rospy.ROSInterruptException
    pass
