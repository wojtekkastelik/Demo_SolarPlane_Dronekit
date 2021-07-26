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

vehicle = connect(connection_string, wait_ready=True, baud=57600)

print("#################################################")
print("Drone reached, switching mode to GUIDED")
print("#################################################")
print("")

vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_currentwp = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_param5 = float(linearray[8])
                ln_param6 = float(linearray[9])
                ln_param7 = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2,
                              ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist



def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    # Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

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

#vehicle.armed = True
time.sleep(2)

print("#################################################")
print("Armed: %s" % vehicle.armed)
print("#################################################")
print("")


print("#################################################")
print("Land!")
print("#################################################")
print("")

vehicle.mode = VehicleMode("LAND")
time.sleep(1)

print("#################################################")
print('Mode: %s' % vehicle.mode)
print("#################################################")
print("")


time.sleep(2)

print("#################################################")
print("Disarming")
print("#################################################")
print("")

# Upload mission from file
import_mission_filename = 'test.waypoints'
upload_mission(import_mission_filename)

# Disarm the vehicle
#vehicle.armed = False
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