#!/usr/bin/python3

"""
.. module::goal_reaching
	:platform: Unix
  	:synopsis:Autonomous driving

Subscribes to:
	/mode to retrieve the current modality

Publishes to:
	/timeout 
	/mode to change the modality if the user wants to quit

This node implements the first modality i.e. the autonomous navigation towards a certain position.
This script, as the name may suggest, implements the 'Autonomous Driving modality'. The user will be asked to insert the 'x' and 'y' coordinates to which the robot will navigate. 
The whole task is accomplished by a *ROS action*. A 60 seconds timeout is set, so if the request cannot be accomplished the goal will be cancelled. 
The user can also cancel the goal before the time is over, it is sufficient to return to the 'idle' status by pressing '0' on the UI console.

"""



import rospy
import time
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *
from std_msgs.msg import Int32, Bool,Float32
from geometry_msgs.msg import Vector3


class colors:
	"""
	Class used for printing colors on the terminal
	"""
	PINK = '\033[95m'
	BLUE = '\033[94m'
	CYAN = '\033[96m'
	GREEN = '\033[92m'
	YELLOW = '\033[93m' 
	RED = '\033[91m'
	ORANGE = '\033[33m' 
	PURPLE  = '\033[35m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'


msg1 = "Goal Reaching Modality!"
msg2 = "(Press '1' from 'user_interface' console to start this modality)"


currentmode=0
done_cb=False #variable which states the accomplishment of the goal
goal_set=False #variable which states if the goal has already been set
isTimeout=False
x_des= 0.0
y_des= 0.0
pos_received=False

def goalpos_callback(v):
	"""
	Callback function which is called when the goal position has been set

	"""
	global  x_des, y_des, pos_received
	x_des= v.x
	y_des= v.y
	pos_received=True
	
def callback_active(): 
	"""
	Callback function which is called when the action starts

	"""
	rospy.loginfo("\nAction server is processing the goal...")

def callback_done(state, result):
	"""
	Callback function which is called when the action is over

	Args:
		state(actionlib_msgs/GoalStatus): status of the goal, the message is of type 'actionlib_msgs/GoalStatus.msg'
		result(MoveBaseResult):result of the goal 

	"""
	global done_cb
	global goal_set
	if state == 3:
		print(colors.GREEN + colors.UNDERLINE + colors.BOLD + "Goal successfully achieved" + colors.ENDC)
		done_cb = True
		return
	if state == 2:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"PREEMPTED"+ colors.ENDC)
		time.sleep(3)
		os.system('cls||clear') #clear the console
		print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+msg2+colors.ENDC)
		return
	if state == 4:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"ABORTED"+ colors.ENDC)
		return
	if state == 5:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"REJECTED"+ colors.ENDC)
		return
	if state == 6:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"PREEMPTING"+ colors.ENDC)
		return
	if state == 7:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"RECALLING"+ colors.ENDC)
		return
	if state == 8:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"RECALLED"+ colors.ENDC)
		return
	if state == 9:
		print(colors.RED + colors.UNDERLINE + colors.BOLD +"LOST"+ colors.ENDC)
		return

def callback_feedback(feedback):
	"""
	Callback function which is called during the execution of the action

	Args:
		feedback(move_base_msgs/MoveBaseActionFeedback.msg): contains infos about the Pose with reference coordinate and timestamp

	"""
	rospy.loginfo("Feedback:%s" % str(feedback))

def set_action(): 
	"""
	Set-up of the action on the client-side

	"""
	global client 
	global goal 
	
	client = actionlib.SimpleActionClient('/move_base',MoveBaseAction) #defining the client
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.orientation.w = 1.0

def set_goal(goal_x_coord,goal_y_coord): 
	"""
	Function to set the goal

	Args:
		goal_x_coord(double): 'x' coordinate of the goal that has to be achieved
		goal_y_coord(double): 'y' coordinate of the goal that has to be achieved

	"""
	global goal_set
	global goal
	os.system('cls||clear') #clear the console
	print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+colors.ENDC)
	goal.target_pose.pose.position.x = goal_x_coord
	goal.target_pose.pose.position.y = goal_y_coord
	print(colors.GREEN + colors.UNDERLINE + colors.BOLD +"Desired Position: ("+str(goal_x_coord)+", "+str(goal_y_coord)+")"+colors.ENDC
		)
	client.send_goal(goal,callback_done,callback_active,callback_feedback) #sending the goal

def my_clbk_timeout(event): 
	"""
	Function to cancel the goal if its time has expired

	Args:
		event(TimerObject): object which includes many infos such as the moment when the callback is called and the duration
	"""
	global isTimeout
	if currentmode==1:
		print (colors.RED + colors.UNDERLINE + colors.BOLD +"Goal time expired")
		isTimeout=True
		
	
def mode_callback(data):
	"""
	Callback function to set the local variable of the current mode if it has been changed by a node

	Args:
		data(int): integer representing the current modality
	"""
	global currentmode
	#rospy.loginfo("I heard %d",data.data)
	currentmode=data.data
    


def main():
	"""
	Main function: if this modality is choosen by the user, it asks the user to insert a given position and therefore sets the action and the goal.
	"""

	global done_cb
	global goal_set
	global isTimeout
	global x_des,y_des
	global pos_received
	
	rospy.init_node('goal_reaching')
	pubTimeout=rospy.Publisher('timeout',Bool,queue_size=10)
	pubModality=rospy.Publisher('mode',Int32,queue_size=10)
	subModality=rospy.Subscriber('mode', Int32,mode_callback)
	subGoalPos=rospy.Subscriber('goalpos', Vector3 , goalpos_callback)
	set_action()
	print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+msg2+colors.ENDC)
	
	while(1):

		if currentmode==1: #if the current mode is '1' i.e. the mode for reaching a goal
			
			if  goal_set==False : #if the goal has not been set yet
				
				#print(colors.UNDERLINE + colors.BOLD +"Where do you want the robot to go?"+colors.ENDC)
				#goal_x_coord = float(input(colors.BOLD +"Insert the 'x' coordinate of the goal: "+colors.ENDC))
				#goal_y_coord = float(input(colors.BOLD +"Insert the 'y' coordinate of the goal: "+colors.ENDC))				
				if pos_received:	
					set_goal(x_des,y_des)	#set the goal
					goal_set = True
					rospy.Timer(rospy.Duration(60),my_clbk_timeout,True)
					pos_received=False
					os.system('cls||clear') #clear the console
			if isTimeout:
				#pubTimeout.publish(True)
				pubModality.publish(0)
				isTimeout=False
				


		else:	#current mode!=1
			
			if goal_set and done_cb==False: #if the goal has been set, the target hasn't been reached yet but the mode has been changed
				client.cancel_goal()
				print (colors.BLUE + colors.UNDERLINE + colors.BOLD +msg1+msg2+colors.ENDC)
			if done_cb: #if the mode has been changed and the task is done
				done_cb=False
			goal_set= False
	rate.sleep()
					
			
     

if __name__ == '__main__':
	main()
        
        
        
        
        
        
