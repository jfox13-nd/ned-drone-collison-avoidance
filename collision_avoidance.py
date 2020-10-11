#!/ usr / bin / env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
Added code for flying using NED Velocity Vectors - Jane Cleland-Huang 1/15/18
"""
import math
import os
import time
import geopy
from math import sin, cos, atan2, radians, sqrt, hypot
from random import randrange

from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
from dronekit_sitl import SITL

from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_utilities import ned_controller, Nedvalues

class Drone(Object):

    def __init__(self, vehicle, ned):
        self.ned = ned
        self.vehicle = vehicle
        self.target = Nedvalues()

    def get_magnitude(self):
        return sqrt( (self.ned.north**2) + (self.ned.east**2) + (self.ned.down**2) )
    
    def random_nudge(self):
        mag = self.get_magnitude()
        nudge_mag = mag / 10
        vector_to_nudge = randrange(3)

        new_ned = Nedvalues()
        new_ned.north = self.ned.north
        new_ned.east = self.ned.east
        new_ned.down = self.ned.down
        neg_or_pos = randrange(1)
        if neg_or_pos == 0:
            neg_or_pos = -1

        if vector_to_nudge == 0:
            new_ned.north += neg_or_pos * nudge_mag

        return new_ned
            
        

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
    print("Reached here")
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


def get_3d_distance_meters(location_a: LocationGlobalRelative, location_b: LocationGlobalRelative) -> float:
    horizontal_distance = get_distance_meters(location_a,location_b)
    vertical_distance = abs(location_a.alt - location_b.alt)
    return hypot(horizontal_distance, vertical_distance)


################################################################################################
# function:    Get distance in meters
# parameters:  Two global relative locations
# returns:     Distance in meters
################################################################################################
def get_distance_meters(locationA, locationB):
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

    # print("Distance (meters):", distance)
    return distance


def drone_vector(drone_a, drone_b):
    opposite_direction_vector = nedcontroller.setNed(drone_b.vehicle.global_relative_frame, drone_a.vehicle.global_relative_frame)


############################################################################################
# Main functionality: Example of one NED command
############################################################################################
drones = []

vehicle, sitl = connect_virtual_vehicle(1,([41.714436,-86.241713,0])) 
drones.append(Drone(vehicle,Nedvalues()))

vehicle, sitl = connect_virtual_vehicle(1,([41.714500,-86.241600,0])) 
drones.append(Drone(vehicle,Nedvalues()))

for drone in drones:
    arm_and_takeoff(10, drone.vehicle)


startingLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
targetLocation = LocationGlobalRelative(41.714500,-86.241600,0)
totalDistance = get_distance_meters(startingLocation,targetLocation)
print("Distance to travel " + str(totalDistance))

# Establish an instance of ned_controller
nedcontroller = ned_controller()

# Get the NED velocity vector mavlink message
ned = nedcontroller.setNed(startingLocation, targetLocation)
nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, vehicle)

# Just wait for it to get there!
print("ARE WE THERE YET?")
currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
while get_distance_meters(currentLocation,targetLocation) > .05:
    currentLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    distance_to_target = get_distance_meters(currentLocation, targetLocation)
    print ('Distance:  {0}  Ground speed: {1} '.format(distance_to_target,vehicle.groundspeed))
    #print (distance_to_target)

# How close did it get?
endLocation = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
print("Starting position: " + str(startingLocation.lat) + ", " + str(startingLocation.lon))
print("Target position: " + str(targetLocation.lat) + ", " + str(targetLocation.lon))
print("End position: " + str(endLocation.lat) + ", " + str(endLocation.lon))

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

