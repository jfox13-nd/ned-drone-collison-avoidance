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
import animation

from dronekit import Vehicle, connect, VehicleMode, LocationGlobalRelative
from dronekit_sitl import SITL

from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_utilities import ned_controller, Nedvalues

R_EARTH = 6378.137
MIN_DISTANCE = 5       # minimum separation distance 

class Drone:

    nedcontroller = ned_controller()

    def __init__(self, vehicle, ned):
        self.ned = ned
        self.vehicle = vehicle
        self.target = Nedvalues()
        self.nudge = None
        self.ontrack = False

    def get_magnitude(self):
        return sqrt( (self.ned.north**2) + (self.ned.east**2) + (self.ned.down**2) )
    
    def set_random_nudge(self):
        mag = self.get_magnitude()
        nudge_mag = mag / 100
        vector_to_nudge = randrange(2)

        new_ned = Nedvalues()
        new_ned.north = self.ned.north
        new_ned.east = self.ned.east
        new_ned.down = self.ned.down
        neg_or_pos = randrange(1)
        if neg_or_pos == 0:
            neg_or_pos = -1

        if vector_to_nudge == 0:
            new_ned.north += neg_or_pos * nudge_mag
        else: 
            new_ned.east += neg_or_pos * nudge_mag

        self.nudge = new_ned

    def drone_vector(self, drone_b):
        ''' this method returns a ned vector for drone a that avoids collision of a drone b '''
        dv = Drone.nedcontroller.setNed(drone_b.vehicle.location.global_relative_frame, self.vehicle.location.global_relative_frame)
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
        self.ned = ned
        Drone.nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, self.vehicle)

    def send_ned(self, ned): 
        ''' sends the drone to a new velocity vector using NED '''
        Drone.nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, 1, self.vehicle)

    def send_to_location(self, location):
        ''' sends drone to location using NED '''
        self.set_and_send_ned(Drone.nedcontroller.setNed(self.vehicle.location.global_relative_frame, location))

    def within_range(self, drone_b): 
        ''' this method checks to see if our drones are within the minimum separation distance '''
        return get_distance_meters(self.vehicle.location.global_relative_frame, drone_b.vehicle.location.global_relative_frame) < MIN_DISTANCE        

    def combine_ned(self, ned, added_ned): 
        ned.north += added_ned.north
        ned.east += added_ned.east
        ned.down += added_ned.down

    def avoid_collision(self, drone_b): 
        ''' call this repeatedly in main to check if drones are within range '''
        if self.within_range(drone_b):      # if the two drones are within range...
            self.ontrack = False
            if not self.nudge:                  # if no nudge has yet been set...
                self.set_random_nudge()             # set a random nudge 
            new_ned = Nedvalues()
            self.combine_ned(new_ned, self.nudge)
            self.combine_ned(new_ned, self.ned)
            # self.combine_ned(new_ned, self.drone_vector(drone_b))
            self.send_ned(new_ned) 


        else: 
            self.nudge = None               # if out of range, no longer need a nudge 
            if not self.ontrack: 
                self.send_ned(self.ned)         # reroute to the OG ned 
                self.ontrack = True
        

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


# def get_3d_distance_meters(location_a: LocationGlobalRelative, location_b: LocationGlobalRelative) -> float:
#     horizontal_distance = get_distance_meters(location_a,location_b)
#     vertical_distance = abs(location_a.alt - location_b.alt)
#     return hypot(horizontal_distance, vertical_distance)


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


def get_coords(latitude, longitude, dx, dy):
    dx = dx / 1000
    dy = dy / 1000
    new_latitude = latitude + (dy / R_EARTH) * (180 / math.pi)
    new_longitude = longitude + (dx / R_EARTH) * (180 / math.pi) / math.cos(latitude * math.pi/180)
    return [new_latitude, new_longitude, 0]

############################################################################################
# Main functionality: Example of one NED command
############################################################################################
drones = []
coordinates = []

#starting_coords = [41.714436,-86.241713,0]
#test_coords_1 = get_coords(starting_coords[0], starting_coords[1], 40, 0)

starting_coords = [41.714841, -86.241941, 0]
test_coords_1 = [41.715092, -86.241546, 0]

test_coords_2 = get_coords(starting_coords[0], starting_coords[1], 0, 40)
test_coords_3 = get_coords(starting_coords[0], starting_coords[1], 28, 28)

vehicle, sitl = connect_virtual_vehicle(1,(starting_coords)) 
drones.append(Drone(vehicle,Nedvalues()))

vehicle, sitl = connect_virtual_vehicle(2,(test_coords_1)) 
drones.append(Drone(vehicle,Nedvalues()))

for drone in drones:
    arm_and_takeoff(10, drone.vehicle)

time.sleep(10)

drones[0].send_to_location(Location(test_coords_1[0], test_coords_1[1]))
drones[1].send_to_location(Location(starting_coords[0], starting_coords[1]))

start_time = time.time()
while time.time() - start_time < 15: 

    # save coordinates to plot 
    coordinates.append([(drones[0].vehicle.location.global_relative_frame.lat, 
        drones[0].vehicle.location.global_relative_frame.lon), 
        (drones[1].vehicle.location.global_relative_frame.lat, 
        drones[1].vehicle.location.global_relative_frame.lon)])

    drones[0].avoid_collision(drones[1])
    drones[1].avoid_collision(drones[0])

print(coordinates)
animation.animate_points(coordinates)





'''
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
'''

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

