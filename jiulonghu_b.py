# -*-coding:utf8 -*-
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
#import exceptions
import math
import argparse
import serial                   #串口
import threading

opponent_state = 0
this_vehicle_state = 0
def re_message(): #接收消息
    global opponent_state
    while 1:
        data=s.readline() #一次接收1024字节1            s.recv    s为和数传进行连接
        postion=data.decode("utf8",'ignore')
        postion.strip() #方法用于移除字符串头尾指定的字符（默认为空格或换行符）
        b=postion.split('  ') #分隔
        #打印信息
        opponent_state=int(b[0])#1号机的this_vehicle_state
        opponent_gps=b[1]
        opponent_velocity=b[2]
        print ("Zhao vehicle state: %s" % opponent_state)
        print ("Zhao vehicle GPS: %s" % opponent_gps)
        print ("Zhao vehicle velocity: %s" % opponent_velocity)

def send_message():
    global this_vehicle_state
    while 1:
        time.sleep(1)
        print ("B battery: %s" % vehicle.battery)
        print("B vehicle state: %s" % this_vehicle_state)
        global_frame=vehicle.location.global_frame
        velocity=vehicle.velocity
        print("B vehicle state: %s" % vehicle.velocity)
        data=str(this_vehicle_state) + "  " + str(global_frame) + "  " + str(velocity) + "\n"
        s.write(data.encode()) #写进s中

def arm_and_takeoff(targetHeight):  #飞机初始化起飞
    #1、飞机自检
    while vehicle.is_armable!=True:  
        print("Waiting for vehicle to become aramable")
        time.sleep(1)
    print("Vehicle is now armable")
    #2、确定飞机模式
    vehicle.mode = VehicleMode("GUIDED")
 
    while vehicle.mode!="GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
    #3、飞机解锁
    vehicle.armed = True
#    while vehicle.armed==False:
#        print("Waiting for drone to become armed")
    time.sleep(3)
#    print("Look out! Virtual props are spinning!")
    #4、飞行
    vehicle.simple_takeoff(targetHeight) ##meters
    #5、打印信息
    while True:
        print("Current Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!!")
    return None
 
 
def get_distance_meters(targetLocation, currentLocation):#获取定点距离
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
 
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5
 
def goto(targetLocation):
    #到定点
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


#1、连接
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600)          #jetson通过串口与飞控连接
print("connect ......     .....            ....     ")
print("connect ......     .....            ....     ")
print("connect ......     .....            ....     ")
s=serial.Serial('/dev/ttyTHS0',115200,8,'N',1)                          #jetson通过串口和数传连接
print("connect the ttl ...... .......")
print("connect the ttl ...... .......")
print("connect the ttl ...... ......")
#2、发送接收线程
re = threading.Thread(target=re_message)
re.start()
se = threading.Thread(target=send_message)
se.start()
#3、设置点位
wp1 = LocationGlobalRelative(31.89281552,118.80898284, 12)
wp2 = LocationGlobalRelative(31.89104090,118.80857005, 12)
wp3 = LocationGlobalRelative(31.89288728,118.80975142, 12)

while 1:
if opponent_state == 1 : #第一台飞机发指令
    arm_and_takeoff(12) #起飞
    vehicle.groundspeed = 6
	goto(wp1)
	goto(wp2)
	vehicle.groundspeed = 0
	i=0
	while i<=10:
		send_local_ned_velocity(-0.2,0,0)
		time.sleep(1)
		print("Moving TRUE NORTH")
		i += 1
    #2号到达指定点后this_vehicle_state=1   给1号发送到opponent_state，1号机降落
    this_vehicle_state = 1
	vehicle.groundspeed = 5
	goto(wp3)
    time.sleep(10)#让1号先降落
    vehicle.mode = VehicleMode("LAND")#降落
    print("LAND mode send to ardupilot........")
    while vehicle.mode != 'LAND':
        print("Waiting for drone to enter LAND mode")
        time.sleep(1)
    print("Vehicle in LAND mode")
    break
#while 1:
#    if opponent_high > 8 :
#        arm_and_takeoff(10)
#    if vehicle.location.global_relative_frame.alt > 8 :
#        if opponent_high < 4 :
#            vehicle.mode = VehicleMode("LAND")
#            while vehicle.mode != 'LAND':
#                print("Waiting for drone to enter LAND mode")
#                time.sleep(1)
##            print("Vehicle in LAND mode")
