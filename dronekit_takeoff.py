from __future__ import print_function
from dronekit import connect, Command, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import time, math

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Demonstrates drones takeoff and land.')
parser.add_argument('--connect',
                    default='/dev/serial0')
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print("#################################################")
print('Connecting to vehicle on: %s' % connection_string)
print("#################################################")
print("")

vehicle = connect(connection_string, wait_ready=True, baud=921600)

print("#################################################")
print("Drone reached, switching mode to GUIDED")
print("#################################################")
print("")

vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

print("#################################################")
print("Telemetry")
print("#################################################")
print("")

print('Mode: %s' % vehicle.mode)
print("Autopilot Firmware version: %s" % vehicle.version)
print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print("Attitude: %s" % vehicle.attitude)
print("Velocity: %s" % vehicle.velocity)
print("GPS: %s" % vehicle.gps_0)
print("Is Armable?: %s" % vehicle.is_armable)
print("Vehicle ekf ok: %s" % vehicle.ekf_ok)

print("")
print("#################################################")
print("")

print("#################################################")
print("Arming motors")
print("#################################################")
print("")

vehicle.armed = True
time.sleep(2)

print("#################################################")
print("Armed: %s" % vehicle.armed)
print("#################################################")
print("")

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Check that vehicle is armable.
    # This ensures home_location is set (needed when saving WP file)
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode

    # time.sleep(10)
    vehicle.armed = True
    while not vehicle.armed:
       print(" Waiting for arming...")
       time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("")
            print("#################################################")
            print("Reached target altitude")
            print("#################################################")
            print("")
            break
        time.sleep(1)


print("#################################################")
print("Take off!")
print("#################################################")
print("")

arm_and_takeoff(15)

time.sleep(2)

print("#################################################")
print("RTL!")
print("#################################################")
print("")

vehicle.mode = VehicleMode("RTL")
time.sleep(1)

print("#################################################")
print('Mode: %s' % vehicle.mode)
print("#################################################")
print("")

while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    # Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt <= 1 * 0.95:
        print("")
        print("#################################################")
        print("Reached rtl")
        print("#################################################")
        print("")
        break
    time.sleep(1)

time.sleep(2)

print("#################################################")
print("Disarming")
print("#################################################")
print("")

# Disarm the vehicle
vehicle.armed = False
time.sleep(2)
# Print the armed state for the vehicle
print("#################################################")
print("Armed: %s" % vehicle.armed)
print("#################################################")
print("")

# Close vehicle object before exiting script
print("#################################################")
print("Close vehicle object")
print("#################################################")
print("")

vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()