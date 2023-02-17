from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse
import serial
import threading

opponent_state = 0
this_vehicle_state = 0
def re_message():
    global opponent_state
    while True:
        data=s.readline()
        postion=data.decode("utf8",'ignore')
        postion.strip()
        b=postion.split('  ')
        opponent_state=int(b[0])#2号机的this_vehicle_state
        opponent_gps=b[1]
        opponent_velocity=b[2]
        print ("B vehicle state: %s" % opponent_state)
        print ("B vehicle GPS: %s" % opponent_gps)
        print ("B vehicle velocity: %s" % opponent_velocity)

def send_message():
    global this_vehicle_state
    while True:
        time.sleep(1)
        print("Zhao Local Location: %s" % vehicle.location.local_frame)
        print("Zhao Attitude: %s" % vehicle.attitude)
        print("Zhao Battery: %s" % vehicle.battery)
        print("Zhao vehicle state: %s" % this_vehicle_state)
        global_frame=vehicle.location.global_frame
        velocity=vehicle.velocity
        data=str(this_vehicle_state) + "  " + str(global_frame) + "  " + str(velocity) + "\n"
        s.write(data.encode())

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become aramable")
        time.sleep(1)
    print("Vehicle is now armable")
 
    vehicle.mode = VehicleMode("GUIDED")
 
    while vehicle.mode!="GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
 
    vehicle.armed = True
#    while vehicle.armed==False:
#        print("Waiting for drone to become armed")
    time.sleep(3)
#    print("Look out! Virtual props are spinning!")
 
    vehicle.simple_takeoff(targetHeight) ##meters
 
    while True:
        print("Current Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")
    return None
 
 
def get_distance_meters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
 
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5
 
def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
 
    vehicle.simple_goto(targetLocation)
 
    while vehicle.mode.name == "GUIDED":
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
        if currentDistance < distanceToTargetLocation*.01:
            print("Reached target waypoint")
            time.sleep(2)
            break
        time.sleep(1)
    return None
def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111000111,
        0,0,0,
        vx,vy,vz,
        0,0,0,
        0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


#1、连接飞控
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)
print("connect ardupilot.....            ....     ")
print("connect ardupilot.....            ....     ")
print("connect ardupilot.....            ....     ")
#2、连接数传
s = serial.Serial('/dev/ttyTHS0',115200,8,'N',1)
print("connect the ttl.....            ....     ")
print("connect the ttl.....            ....     ")
print("connect the ttl.....            ....     ")
re = threading.Thread(target=re_message)
re.start()
se = threading.Thread(target=send_message)
se.start()

wp1 = LocationGlobalRelative(31.89281552,118.80898284, 15)
wp2 = LocationGlobalRelative(31.89104090,118.80857005, 15)
wp3 = LocationGlobalRelative(31.89288728,118.80965142, 15)
arm_and_takeoff(15)
vehicle.groundspeed = 5
goto(wp1)
goto(wp2)
vehicle.groundspeed = 0
#到达指定点后this_vehicle_state=1   给2号发送到opponent_state，2号接收到后起飞
this_vehicle_state = 1
#1号机在等2号到来
i=0
while i<=150:
	if opponent_state == 0 :
		send_local_ned_velocity(-0.2,0,0)
		time.sleep(1)
		print("Moving TRUE NORTH")
		i += 1
	elif opponent_state == 1 :
		print("Go Back")
		vehicle.groundspeed = 5
		#lat=31.8908519,lon=118.8085763,alt=12.009
		goto(wp3)
		break
time.sleep(1)	
while 1:
#2号机到达后传过来的指令
if opponent_state == 1 :
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode != 'LAND':
        print("Waiting for drone to enter LAND mode")
    time.sleep(1)
    print("Vehicle in LAND mode")

