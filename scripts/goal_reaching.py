#!/usr/bin/python3

# including required libraries 
import rospy
import time
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *
from std_msgs.msg import Int32, Bool,Float32
from geometry_msgs.msg import Vector3

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
	Callback function to set goal position

	"""
	global  x_des, y_des, pos_received
	x_des= v.x
	y_des= v.y
	pos_received=True
	
def callback_active(): 
	"""
	Callback function to starts action

	"""
	rospy.loginfo("\nAction server is processing the goal...")

def callback_done(state, result):
	"""
	Callback function when action is finished 

	
	"""
	global done_cb
	global goal_set
	if state == 3:
		print("Goal successfully achieved")
		done_cb = True
		return
	if state == 2:
		print("PREEMPTED")
		time.sleep(3)
		os.system('cls||clear') #clear the console
		print (msg1+msg2)
		return
	if state == 4:
		print("ABORTED")
		return
	if state == 5:
		print("REJECTED")
		return
	if state == 6:
		print("PREEMPTING")
		return
	if state == 7:
		print("RECALLING")
		return
	if state == 8:
		print("RECALLED")
		return
	if state == 9:
		print("LOST")
		return

def callback_feedback(feedback):
	"""
	Callback function for action execution

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

	"""
	global goal_set
	global goal
	os.system('cls||clear') #clear the console
	print (msg1)
	goal.target_pose.pose.position.x = goal_x_coord
	goal.target_pose.pose.position.y = goal_y_coord
	print("Desired Position: ("+str(goal_x_coord)+", "+str(goal_y_coord)+")")
	client.send_goal(goal,callback_done,callback_active,callback_feedback) #sending the goal

def my_clbk_timeout(event): 
	"""
	Function to cancel the goal when time expired

	"""
	global isTimeout
	if currentmode==1:
		print ("Goal time expired")
		isTimeout=True
		
	
def mode_callback(data):
	"""
	Callback function to set the local variable of the current mode if it has been changed by a node
	
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
	print (msg1+msg2)
	
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
				print (msg1+msg2)
			if done_cb: #if the mode has been changed and the task is done
				done_cb=False
			goal_set= False
	rate.sleep()
					
			
     

if __name__ == '__main__':
	main()
        
        
        
        
        
        
