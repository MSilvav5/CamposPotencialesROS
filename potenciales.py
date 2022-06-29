#!/usr/bin/env python
#Obtain in Manuel Silva Verastegui github
import rospy
import tf
import time
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion,quaternion_from_euler


vx = vth = xrob = yrob = 0
xg = 6
yg = 6
umbral = 1.8
Katt = 7
Krep = 0.009
twist = Twist()


def lidar(msg):
	global laser_ranges
	global laser_angles
	global Xrep
	global Yrep
	laser_ranges = []
	laser_angles = []
	f = s = F = S = 0

	laser_ranges = msg.ranges
	
	for i in range(0,len(laser_ranges)):
		laser_angles.append(msg.angle_min+i*msg.angle_increment)

	for i in range(0, len(laser_ranges)):
		if (math.isinf(laser_ranges[i])==False):
			f = (1/((laser_ranges[i])**2))*math.cos(laser_angles[i])
			s = (1/((laser_ranges[i])**2))*math.sin(laser_angles[i])
			F=F+f
			S=S+s

	Xrep=Krep*F*math.cos(thetarob)-Krep*S*math.sin(thetarob)
	Yrep=Krep*F*math.sin(thetarob)+Krep*S*math.cos(thetarob)
	
def callback(msg):
	global R
	global thetarob
	global Xatt
	global Yatt
	p = msg.pose.pose 
	xrob = p.position.x
	yrob = p.position.y
	Xatt = xg-xrob
	Yatt = yg-yrob
	R = np.hypot(Xatt , Yatt)
	orientation_q = msg.pose.pose.orientation
	orientation_list=[orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll,pitch,thetarob)=euler_from_quaternion(orientation_list)
     
def main(R):
	Px=Xatt-Xrep
	Py=Yatt-Yrep
	P=np.hypot(Px,Py)
	thetap=math.atan2(Py,Px)
	
	if R > umbral:
		Rmax = umbral
		Patt = Katt*Rmax
		twist.angular.z=1*(thetap-thetarob)
		pub.publish(twist)
		time.sleep(0.5)
		twist.angular.z=0
		pub.publish(twist)
		twist.linear.x=0.05*Patt
		pub.publish(twist)
		print("FUERA de umbral")
	else: 
		Patt = Katt*R
		twist.angular.z=1*(thetap-thetarob)
		pub.publish(twist)
		time.sleep(0.5)
		twist.angular.z=0
		pub.publish(twist)
		twist.linear.x=0.05*Patt
		pub.publish(twist)
		print("DENTRO de umbral")


	print("FUERZAS",Xatt,Yatt,Xrep,Yrep)
	print("ANGULOS",thetap,thetarob)

if __name__ == '__main__':

	while not rospy.is_shutdown():
		rospy.init_node('Artificial_PF', anonymous=True)
		rospy.Subscriber('/p3dx/odom', Odometry, callback)
		rospy.Subscriber("/p3dx/laser/scan", LaserScan, lidar)
		pub = rospy.Publisher("/p3dx/cmd_vel", Twist, queue_size=10)
		time.sleep(0.1) 
		if R > 0.2:
			main(R)
		else:
			print("Llegamos")
			twist.linear.x=0
			pub.publish(twist)
