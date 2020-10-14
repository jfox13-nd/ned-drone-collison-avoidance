#!/ usr / bin / env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
Added code for flying using NED Velocity Vectors - Jane Cleland-Huang 1/15/18
"""
import math
import os
import time
from math import sin, cos, atan2, radians, sqrt, hypot
from random import randrange
import animation
import sys

from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
from dronekit_sitl import SITL

from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_utilities import ned_controller, Nedvalues

R_EARTH = 6378.137
MIN_DISTANCE = 5       # minimum separation distance 

class Drone:

    def __init__(self, vehicle, ned, nedcontroller, speed):
        self.ned = ned
        self.vehicle = vehicle
        self.target = Nedvalues()
        self.nudge = None
        self.nedcontroller = nedcontroller
        self.speed = speed

    def get_magnitude(self):
        ''' returns the magnitude of the current NED '''
        return sqrt( (self.ned.north**2) + (self.ned.east**2) + (self.ned.down**2) )

    def get_ned_mag(self, ned_for_mag):
        ''' Return a magnitude given ned values '''
        return sqrt( (ned_for_mag.north**2) + (ned_for_mag.east**2) + (ned_for_mag.down**2) )

    def drone_vector(self, drone_b):
        ''' this method returns a ned vector for drone a that avoids collision of a drone b '''
        dv = self.nedcontroller.setNed(drone_b.vehicle.location.global_relative_frame, self.vehicle.location.global_relative_frame)
        new_mag = sqrt( (dv.north**2) + (dv.east**2) + (dv.down**2) )
        dv.north /= new_mag
        dv.east /= new_mag
        dv.down /= new_mag

        dv.north *= self.get_magnitude()
        dv.east *= self.get_magnitude()
        dv.down *= self.get_magnitude()
        return dv

    def set_and_send_ned(self, ned): 
        ''' gives the drone a new velocity vector using NED + sets its internal ned attribute '''
        m = self.get_ned_mag(ned)
        ned.north /= m
        ned.east /= m
        ned.down /= m
        ned.north *= self.speed
        ned.east *= self.speed
        ned.down *= self.speed
        self.ned = ned
        self.nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, self.vehicle)

    def send_ned(self, ned): 
        ''' sends the drone to a new velocity vector using NED '''
        m = self.get_ned_mag(ned)
        ned.north /= m
        ned.east /= m
        ned.down /= m
        ned.north *= self.speed
        ned.east *= self.speed
        ned.down *= self.speed
        self.nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, self.vehicle)

    def send_to_location(self, location):
        ''' sends drone to location using NED '''
        self.set_and_send_ned(self.nedcontroller.setNed(self.vehicle.location.global_relative_frame, location))

    def within_range(self, drone_b): 
        ''' this method checks to see if our drones are within the minimum separation distance '''
        return get_distance_meters(self.vehicle.location.global_relative_frame, drone_b.vehicle.location.global_relative_frame) < MIN_DISTANCE        

    def combine_ned(self, ned, added_ned):
        ''' Add two NED vectors together '''
        ned.north += added_ned.north
        ned.east += added_ned.east
        ned.down += added_ned.down

    def avoid_collision(self, drone_b): 
        ''' call this repeatedly in main to check if drones are within range '''
        if self.within_range(drone_b):      # if the two drones are within range...
            new_ned = Nedvalues()
            self.combine_ned(new_ned, self.ned)
            self.combine_ned(new_ned, self.drone_vector(drone_b))
            self.send_ned(new_ned)
        else: 
            self.nudge = None               # if out of range, no longer need a nudge 
            self.send_ned(self.ned)         # reroute to the OG ned 

            time.sleep(0.005)
        

################################################################################################
# Standard Connect
################################################################################################
def connect_virtual_vehicle(instance, home):
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    instance_arg = '-I%s' %(str(instance))
    home_arg = '--home=%s, %s,%s,180' % (str(home[0]), str(home[1]), str(home[2]))
    sitl_args = [instance_arg, '--model', 'quad', home_arg]
    sitl.launch(sitl_args, await_ready=True)
    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + instance * 10)
    conn_string = ':'.join([tcp, ip, port])
    print('Connecting to vehicle on: %s' % conn_string)

    vehicle = connect(conn_string)
    vehicle.wait_ready(timeout=120)
    print("Vehicle created")
    return vehicle, sitl


################################################################################################
# ARM and TAKEOFF
################################################################################################

# function:   	arm and takeoff
# parameters: 	target altitude (e.g., 10, 20)
# returns:	n/a

def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("home: " + str(vehicle.location.global_relative_frame.lat))

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .95:
            print("Reached target altitude")
            break
        time.sleep(1)

################################################################################################
# function:    Get distance in meters
# parameters:  Two global relative locations
# returns:     Distance in meters
################################################################################################
def get_distance_meters(locationA, locationB):
    ''' Get the distance in meters between two locations '''
    # approximate radius of earth in km
    R = 6373.0

    lat1 = radians(locationA.lat)
    lon1 = radians(locationA.lon)
    lat2 = radians(locationB.lat)
    lon2 = radians(locationB.lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = (R * c) * 1000

    return distance


def get_coords(latitude, longitude, dx, dy):
    ''' Get the lattitude and longitude coordinates a certain distance in meters away '''
    dx = dx / 1000.0  # convert to meters 
    dy = dy / 1000.0
    new_latitude = latitude + (dy / R_EARTH) * (180 / math.pi)
    new_longitude = longitude + (dx / R_EARTH) * (180 / math.pi) / math.cos(latitude * math.pi/180)
    return [new_latitude, new_longitude, 0]

def usage():
    ''' simple usage function '''
    print("Usage:\n\tpython collision_avoidance.py [1|2|3] speed\n\tspeed: drone speed in m/s. Suggested value = 1")
    sys.exit(0)


############################################################################################
# Main functionality: Collision testing with two drones
############################################################################################
if __name__ == '__main__':
    if len(sys.argv) < 3 or len(sys.argv) > 3 or (sys.argv[1] != '1' and sys.argv[1] != '2' and sys.argv[1] != '3'):
        usage()

    print("Conducting test %s:" % sys.argv[1])
    if sys.argv[1] == '1':
        print("Two drones approach each other from 40 meters")
    elif sys.argv[1] == '2':
        print("Two drones fly parallel with a 5 meter distance between them")
    elif sys.argv[1] == '3':
        print("Two drones fly parallel with a 1 meter distance between them")
    speed = float(sys.argv[2])

    drones = []
    coordinates = []

    starting_coords = [41.714841, -86.241941, 0]
    test_coords_1 = get_coords(starting_coords[0], starting_coords[1], 20, 20)
    test_coords_2 = get_coords(starting_coords[0], starting_coords[1], 0, 5)
    test_coords_3 = get_coords(starting_coords[0], starting_coords[1], 400, 0)
    test_coords_4 = get_coords(starting_coords[0], starting_coords[1], 400, 5)
    test_coords_5 = get_coords(starting_coords[0], starting_coords[1], 0, 1)
    test_coords_6 = get_coords(starting_coords[0], starting_coords[1], 400, 1)

    window_helper_1 = get_coords(starting_coords[0], starting_coords[1], 0, 10)
    window_helper_2 = get_coords(starting_coords[0], starting_coords[1], 0, -5)


    nedcontroller = ned_controller()

    if sys.argv[1] == '1':
        vehicle, sitl = connect_virtual_vehicle(1,(starting_coords)) 
        drones.append(Drone(vehicle,Nedvalues(),nedcontroller,speed))

        targetLocation = LocationGlobalRelative(41.715115, -86.241615, 10) 

        vehicle, sitl = connect_virtual_vehicle(2,(test_coords_1)) 
        drones.append(Drone(vehicle,Nedvalues(),nedcontroller,speed))
    else:
        vehicle, sitl = connect_virtual_vehicle(1,(starting_coords)) 
        drones.append(Drone(vehicle,Nedvalues(),nedcontroller,speed))

        targetLocation = LocationGlobalRelative(41.715115, -86.241615, 10) 
        if sys.argv[1] == '2':
            vehicle, sitl = connect_virtual_vehicle(2,(test_coords_2))
        else:
            vehicle, sitl = connect_virtual_vehicle(2,(test_coords_5))
        drones.append(Drone(vehicle,Nedvalues(),nedcontroller,speed))

    # have drones takeoff and get to an altitude of 10 m
    for drone in drones:
        print("hello")
        arm_and_takeoff(10, drone.vehicle)
    time.sleep(10)

    print("Sending drones")
    if sys.argv[1] == '1':
        drones[0].send_to_location(Location(test_coords_1[0], test_coords_1[1]))
        drones[1].send_to_location(Location(starting_coords[0], starting_coords[1]))
    else:
        drones[0].send_to_location(Location(test_coords_3[0], test_coords_3[1]))
        if sys.argv[1] == '2':
            drones[1].send_to_location(Location(test_coords_4[0], test_coords_4[1]))
        else:
            drones[1].send_to_location(Location(test_coords_6[0], test_coords_6[1]))


    # fly drones for 30 seconds while collecting location data
    start_time = time.time()
    while time.time() - start_time < 30: 

        # save coordinates to plot
        if sys.argv[1] == '1':
            coordinates.append([(drones[0].vehicle.location.global_relative_frame.lat, 
                drones[0].vehicle.location.global_relative_frame.lon), 
                (drones[1].vehicle.location.global_relative_frame.lat, 
                drones[1].vehicle.location.global_relative_frame.lon)])
        else:
            coordinates.append([(drones[0].vehicle.location.global_relative_frame.lat, 
                drones[0].vehicle.location.global_relative_frame.lon), 
                (drones[1].vehicle.location.global_relative_frame.lat, 
                drones[1].vehicle.location.global_relative_frame.lon),
                (window_helper_1[0],window_helper_1[1]),
                (window_helper_2[0],window_helper_2[1])])

        time.sleep(0.005)

        drones[0].avoid_collision(drones[1])
        drones[1].avoid_collision(drones[0])


    print("Number of coordinates collected:")
    print(len(coordinates))

    print("Starting animation")
    animation.animate_points(coordinates)

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()