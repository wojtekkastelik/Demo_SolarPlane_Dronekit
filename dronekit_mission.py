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
home=vehicle.home_location

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


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist = []
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)
    # Download mission from vehicle
    missionlist = download_mission()
    # Add file-format information
    output = 'QGC WPL 110\n'
    # Add home location as 0th waypoint
    home = vehicle.home_location
    output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
    0, 1, 0, 16, 0, 0, 0, 0, home.lat, home.lon, home.alt, 1)
    # Add commands
    for cmd in missionlist:
        commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        cmd.seq, cmd.current, cmd.frame, cmd.command, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.x, cmd.y,
        cmd.z, cmd.autocontinue)
        output += commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip())


import_mission_filename = 'test.waypoints'

# Upload mission from file
upload_mission(import_mission_filename)

print("")
print("#################################################")
print("Upload completed!")
print("#################################################")
print("")

time.sleep(2)

print("#################################################")
print("Take off!")
print("#################################################")
print("")

arm_and_takeoff(10)

time.sleep(2)

print("#################################################")
print("Altitude reached, switching mode to AUTO")
print("#################################################")
print("")

vehicle.mode = VehicleMode("AUTO")
time.sleep(2)

print("#################################################")
print("Starting mission")
print("#################################################")
print("")
#print("Starting mission")
# Reset mission set to first (0) waypoint
#vehicle.commands.next=0
while True:
    nextwaypoint = vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    if nextwaypoint == 2:  # Skip to next waypoint
        print("#################################################")
        print("First waypoint reached, switching mode to GUIDED")
        print("#################################################")
        print("")
        vehicle.mode = VehicleMode("GUIDED")
        break
    time.sleep(1)

#print("zmien mode, masz 5s na rtl")
#vehicle.mode = VehicleMode("RTL")
#time.sleep(3)
time.sleep(1)
print("#################################################")
print('Mode: %s' % vehicle.mode)
print("#################################################")
print("")

print("#################################################")
print('Going to point')
print("#################################################")
print("")
point = LocationGlobalRelative(50.1294203, 19.7878844, 10)
vehicle.simple_goto(point)

while True:
    distance = get_distance_metres(vehicle.location.global_frame, point)
    print('Distance to point: %s' % distance)
    if distance < 1:
        print("#################################################")
        print("Point reached, switching mode to AUTO")
        print("#################################################")
        print("")
        vehicle.mode = VehicleMode("AUTO")
        break
    time.sleep(1)

time.sleep(1)
print("#################################################")
print('Mode: %s' % vehicle.mode)
print("#################################################")
print("")

while True:
    nextwaypoint = vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    if nextwaypoint == 3 && distance_to_current_waypoint() < 1:  # Skip to next waypoint
        print("#################################################")
        print("Third waypoint reached, switching mode to LAND")
        print("#################################################")
        print("")
        vehicle.mode = VehicleMode("LAND")
        break
    time.sleep(1)

time.sleep(1)
print("#################################################")
print('Mode: %s' % vehicle.mode)
print("#################################################")
print("")


time.sleep(1)


while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    # Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt <= 1 * 0.95:
        print("")
        print("#################################################")
        print("Reached land")
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