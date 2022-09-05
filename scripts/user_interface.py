#!/usr/bin/python3

"""
.. module:: user_interface
	:platform: Unix
  	:synopsis: User Interface
..

Subscribes to:
	/timeout to check if the timeout for the autonomous driving mode has expired

Publishes to:
	/mode to start a new modality
	/goalpos to send the coordinates of the goal

The User Interface is the node that lets the user switch between the modalities, including the 'idle' one (i.e. when no mode is active). 
The command is given by a user keyboard input and it is sent to the other nodes using ROS topics.

"""

import rospy
import os
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Vector3
isTimeout=False #variable that'll be set to true if the timeout expires

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

def timeout_callback(data): 
	"""
	Callback function for the timer

	Args:
		data(bool): variable to notice if the timeout (for mode 1) has expired

	"""
	
	global isTimeout
	isTimeout=True
	

def main():
	"""
	Main Function: constantly asks for the user to press a certain key to start or change a modality. When the user presses a proper key (i.e '0','1','2','3','4'), the corresping value will be pushibled on /mode topic. In 'Goal Reaching' mode, a message with the goal position will be also published on /goalpos topic  

	"""

	global isTimeout
	rospy.init_node('user_interface')
	pubModality=rospy.Publisher('mode',Int32,queue_size=10) #publisher of 'mode' topic, sends user choice to other nodes
	pubGoalPos=rospy.Publisher('goalpos',Vector3,queue_size=10) #publisher of the target position
	subTimeout=rospy.Subscriber('timeout', Bool,timeout_callback) #subscriber of 'timeout' topic to receive timeout notification from goal_reaching node
	print(colors.GREEN + colors.UNDERLINE + colors.BOLD + "\nIdle status. Waiting for a command from user..."+colors.ENDC)
	while not rospy.is_shutdown():

		try:
			command = int(input(colors.BLUE + colors.UNDERLINE + colors.BOLD +'\nChoose the modality:\n - [0] Idle,\n - [1] Goal Reaching,\n - [2] Not Assisted Driving,\n - [3] Assisted Driving,\n - [4] Quit \n'+colors.ENDC))

		except ValueError:
			command = -1

		os.system('cls||clear') #clear the console



		if command == 0: #idle status
			
			currentmode=0
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			print(colors.GREEN + colors.UNDERLINE + colors.BOLD + "\nIdle status. Waiting for a command from user..."+colors.ENDC)


		elif command == 1: #first modality (Goal Reaching)

			
			print(colors.UNDERLINE + colors.BOLD +"Where do you want the robot to go?"+colors.ENDC)
			goal_x_coord = float(input(colors.BOLD +"Enter the 'x' coordinate of the goal: "+colors.ENDC))
			goal_y_coord = float(input(colors.BOLD +"Enter the 'y' coordinate of the goal: "+colors.ENDC))
			msg=Vector3()
			msg.x=goal_x_coord
			msg.y=goal_y_coord
			pubModality.publish(1)
			pubGoalPos.publish(msg)
			os.system('cls||clear') #clear the console
			print(colors.BOLD + colors.UNDERLINE +colors.PINK+ "\nModality 1 - Goal Reaching"+ colors.ENDC)
			print( colors.BOLD + "(press '0' during the execution to cancel the target)" + colors.ENDC)
			currentmode=1
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			

		elif command == 2: #second modality (Not Assisted Driving)
			
			currentmode=2
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			print(colors.PURPLE + colors.UNDERLINE + colors.BOLD +"\nModality 2 - Not Assisted Driving\n"+colors.ENDC)
			
			
		elif command == 3: #third modality (Assisted Driving)
			
			currentmode=3
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			print(colors.CYAN + colors.UNDERLINE + colors.BOLD +"\nModality 3 - Assisted Driving\n"+colors.ENDC)
			
		elif command == 4:
			exit()
			
		else:
			print(colors.RED + colors.UNDERLINE + colors.BOLD +"Wrong key"+colors.ENDC)


if __name__ == '__main__':
	main()

