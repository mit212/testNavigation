#! /usr/bin/env python

# 2.12 Final Project
# Phillip Daniel April 2021

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

# These global variables are the pose of the mobile robot
x= 0.0
y=0.0
theta=0.0

def newOdom(msg):
	# Description: This function is called after a message 'msg' is read from the subscribed channel 'sub'
	# Return: 
		# [x,y,theta] - Sets the global variables, which are the pose of the mobile robot with respect to an inertial frame
	# Argument: 
		# msg - A message object/class

	global x
	global y
	global theta

	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y


	rot_q=msg.pose.pose.orientation
	(roll, pitch, theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def searchForAprilTags():
	# Description: This function uses the global pose of the robot to compute whether or not an April tag is visible.
	# Return
		# relPos.relPos_i_.label - The label for the visible April tags (e.g. 'tag1')
		# relPos.relPos_i_.x - The x-axis position of the April tag with respect to (wrt) the coordinate system that is attached to the mobile robot
		# relPos.relPos_i_.y - The y-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
	# Argument: 
		# none

	global x
	global y
	global theta


	# Objects that represent the location of the different April tags in the room, as well as the bounding box that determines when a tag is visible.
	class myClass(object):
		pass
	
	# Tag one
	# Visibility information
	tag1Vis=myClass()
	tag1Vis.label='tag1'
	# Tag Position Relative to Global (0,0)
	tag1Vis.x=-3.85
	tag1Vis.y=-1.79
	# Robot Position Bounding Box
	tag1Vis.xMin=tag1Vis.x
	tag1Vis.xMax=10
	tag1Vis.yMin=-100
	tag1Vis.yMax=100

	
	# Tag two
	# Visibility information
	tag2Vis=myClass()
	tag2Vis.label='tag2'
	# Tag Position Relative to Global (0,0)
	tag2Vis.x=-3.3
	tag2Vis.y=-7.0
	# Robot Position Bounding Box
	tag2Vis.xMin=-7
	tag2Vis.xMax=10
	tag2Vis.yMin=-100
	tag2Vis.yMax=100

	
	# Tag three
	# Visibility information
	tag3Vis=myClass()
	tag3Vis.label='tag3'
	# Tag Position Relative to Global (0,0)
	tag3Vis.x=-6.66
	tag3Vis.y=-6.72
	# Robot Position Bounding Box
	tag3Vis.xMin=tag3Vis.x
	tag3Vis.xMax=0
	tag3Vis.yMin=-8
	tag3Vis.yMax=0

	
	# Tag four
	# Visibility information
	tag4Vis=myClass()
	tag4Vis.label='tag4'
	# Tag Position Relative to Global (0,0)
	tag4Vis.x=-6.23
	tag4Vis.y=-0.19
	# Robot Position Bounding Box
	tag4Vis.xMin=-10
	tag4Vis.xMax=-5.83
	tag4Vis.yMin=-100
	tag4Vis.yMax=100

	
	# Tag five
	# Visibility information
	tag5Vis=myClass()
	tag5Vis.label='tag5'
	# Tag Position Relative to Global (0,0)
	tag5Vis.x=-9
	tag5Vis.y=-4.88
	# Robot Position Bounding Box
	tag5Vis.xMin=-10
	tag5Vis.xMax=-5.83
	tag5Vis.yMin=-4.88
	tag5Vis.yMax=0.0

	

	def computeLineOfSight(tagVis, x, y, theta): 
		# Description: This returns information from each tag that is visible, based on the pose of the robot and the geometric information about the tags.
		# Return
			# relPos.label - The label for the visible April tags (e.g. 'tag1')
			# relPos.x - The x-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
			# relPos.y - The y-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
		# Argument
			# tagVis - Geometric information about the tags
			# x - x position of robot wrt the global coordinate system (CS)
			# y - y position of robot wrt the global CS
			# theta - Orientation of robot wrt the global CS

		#  Robot's field of view
		cameraFOV=1.12
		halfFOV=(cameraFOV)/2 # [Rad]
		
		# Compute relative angle between the April Tag and the robot
		V=np.array([tagVis.x-x, tagVis.y-y]) #Vector from robot to april tag, wrt global frame
		xHat=np.array([np.cos(theta),np.sin(theta)]) #x axis unit vector of robot frame wrt global frame
		yHat=np.array([-np.sin(theta),np.cos(theta)]) #y axis unit vector of robot frame wrt global frame
		vRel=np.array([np.dot(xHat,V), np.dot(yHat,V)]) #Vector from robot to april tag, wrt robot frame (ie camera frame)
		alpha=atan2(np.dot(V,yHat), np.dot(V, xHat)) # Angle between vRel and the +xHat


		# Check if the robot is pointed at the tag, and in the proper bounding box to see the tag
		class relativePositionOfTagFromRobot(object):
			pass

		relPos=relativePositionOfTagFromRobot() 


		if x>tagVis.xMin and x<tagVis.xMax and y>tagVis.yMin and y<tagVis.yMax and ( alpha > -halfFOV and alpha < halfFOV ):
			relPos.label=tagVis.label
			relPos.x=vRel[0]
			relPos.y=vRel[1]
		else:
			relPos.label='no tag'
		return relPos

	relPos = myClass()

	relPos.relPos1=computeLineOfSight(tag1Vis, x, y, theta)
	relPos.relPos2=computeLineOfSight(tag2Vis, x, y, theta)
	relPos.relPos3=computeLineOfSight(tag3Vis, x, y, theta)
	relPos.relPos4=computeLineOfSight(tag4Vis, x, y, theta)
	relPos.relPos5=computeLineOfSight(tag5Vis, x, y, theta)

	return relPos 
	
def setSpeeds(xDot, yDot, thetaDot):
	# Description: Converts the inputs to the desired speed of the robot
	# Return
		# speed.linear.x - Desired speed of robot wrt the x-axis of its body fixed frame [m/s]
		# speed.linear.y  - Desired speed of robot wrt the y-axis of its body fixed frame [m/s]
		# speed.angular.z - Desired angular velocity of robot wrt the z-axis of its body fixed frame [rad/s]
	# Argument
		# xDot - Fraction of the robot's maximum speed along this axis from -1 to 1
		# yDot - Fraction of the robot's maximum speed along this axis from -1 to 1
		# thetaDot - Fraction of the robot's maximum speed along this axis from -1 to 1
	def clamp(num):
		# Description: Restrics the output to a specified range
			# Return
				# num - Value that has been clipped to a specified range
			# Argument
				# num - Scalar value 

		return max(min(num,1),-1)

	xDot=clamp(xDot)
	yDot=clamp(yDot)
	thetaDot=clamp(thetaDot)

	maxSpeed=1.4 #Maximum Linear Speed m/s
	maxRot=.5 #Maximum Angular Velocity rad/s

	speed.linear.x = xDot*maxSpeed
	speed.linear.y = yDot*maxSpeed
	speed.angular.z = thetaDot*maxRot

	return speed

def pointAtTag(tagID):
	# Description: Search for the specified tag
	# Return
		# void - Prints a message when we are pointing at a tag
	# Argument
		# tagID - The ID of the tag that we wish to point the robot's camera towards
	class myClass(object):
			pass
	locatedTag = myClass()

	xDot=0
	yDot=0
	rate=.75
	thetaDot=rate
	locatedTag.pointed = False


	while locatedTag.pointed == False:
		tagInfo = searchForAprilTags()
		locatedTag=viewedTagRelPos(tagID)

		# Spin around and look for the tag 'locatedTag.label.' Stop once we are pointing at it within a small window for error
		if locatedTag.label != 'no tag': # If we have found a tag, it must be that one that we were looking for.
			if locatedTag.relY<.05 and locatedTag.relY>-.05: # If we are pointing at the tag
				thetaDot=0
				locatedTag.pointed = True
			else:
				locatedTag.pointed = False
		else:
			thetaDot=rate
			locatedTag.pointed = False

		speed=setSpeeds(xDot, yDot, thetaDot)
		pub.publish(speed)
		r.sleep()
	print('Pointed at tag')

def viewedTagRelPos(tagID):
	# Description: Relative position of the tag 'tagID,' if it is in view
		# Return
			# locatedTag.label - The label for the visible April tags (e.g. 'tag1')
			# locatedTag.relX - The x-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
			# locatedTag.relY - The y-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
		# Argument
			# tagID - The ID of the tag that we wish to point the robot's camera towards
	class myClass(object):
			pass
	locatedTag = myClass()

	# Search
	tagInfo = searchForAprilTags()
	if tagID == tagInfo.relPos1.label:
		locatedTag.relX = tagInfo.relPos1.x
		locatedTag.relY = tagInfo.relPos1.y
		locatedTag.label=tagInfo.relPos1.label
	elif tagID == tagInfo.relPos2.label:
		locatedTag.relX = tagInfo.relPos2.x
		locatedTag.relY = tagInfo.relPos2.y
		locatedTag.label=tagInfo.relPos2.label
	elif tagID == tagInfo.relPos3.label:
		locatedTag.relX = tagInfo.relPos3.x
		locatedTag.relY = tagInfo.relPos3.y
		locatedTag.label=tagInfo.relPos3.label
	elif tagID == tagInfo.relPos4.label:
		locatedTag.relX = tagInfo.relPos4.x
		locatedTag.relY = tagInfo.relPos4.y
		locatedTag.label=tagInfo.relPos4.label
	elif tagID == tagInfo.relPos5.label:
		locatedTag.relX = tagInfo.relPos5.x
		locatedTag.relY = tagInfo.relPos5.y
		locatedTag.label=tagInfo.relPos5.label
	else:
		locatedTag.label='no tag'

	return locatedTag

def approach(tagID):
	# Description: Drive within a certian distace of the tag 'tagID'
		# Return
			# null
		# Argument
			# tagID - The ID of the tag that we wish to point the robot's camera towards
	viewedTag=viewedTagRelPos(tagID)
	viewedTag.approached=False
	while viewedTag.approached==False:
		viewedTag=viewedTagRelPos(tagID)
		vRel=np.array([viewedTag.relX,viewedTag.relY])
		relPosNorm=np.linalg.norm(vRel)
		relPosUnitVec=vRel/relPosNorm
		thetaDot=0

		if relPosNorm > .2:
			xDot=relPosUnitVec[0]
			yDot=relPosUnitVec[1]
			viewedTag.approached=False
		else:
			xDot=0
			yDot=0
			viewedTag.approached=True

		speed=setSpeeds(xDot, yDot, thetaDot)

		pub.publish(speed)
		r.sleep()
	print('Arrived at tag')

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(100)


# This is the main loop
while not rospy.is_shutdown():
	pointAtTag('tag1')
	approach('tag1')
	pointAtTag('tag2')
	approach('tag2')
	pointAtTag('tag3')
	approach('tag3')
	pointAtTag('tag4')
	approach('tag4')
	pointAtTag('tag5')
	approach('tag5')
	pointAtTag('tag4')
	approach('tag4')
	pointAtTag('tag3')
	approach('tag3')
	pointAtTag('tag2')
	approach('tag2')
	pointAtTag('tag1')
	approach('tag1')
	while 1:
		r.sleep()
