#!/usr/bin/python3

import rospy
import os
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Vector3
isTimeout=False #variable that'll be set to true if the timeout expires

def timeout_callback(data): 
	"""
	Callback function for the timer

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
	print(colors.GREEN + colors.UNDERLINE + colors.BOLD + "\nFree status. Waiting for a command from user..."+colors.ENDC)
	while not rospy.is_shutdown():

		try:
			command = int(input('\nChoose the modality:\n - [0] Free,\n - [1] Goal Reaching,\n - [2] Not Assisted Driving,\n - [3] Assisted Driving,\n - [4] Quit \n'))

		except ValueError:
			command = -1

		os.system('cls||clear') #clear the console



		if command == 0: #free status
			
			currentmode=0
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			print("\nfree status. Waiting for a command from user...")


		elif command == 1: #first modality (Goal Reaching)

			
			print("Where do you want the robot to go?")
			goal_x_coord = float(input("Insert the 'x' coordinate of the goal: "))
			goal_y_coord = float(input("Insert the 'y' coordinate of the goal: "))
			msg=Vector3()
			msg.x=goal_x_coord
			msg.y=goal_y_coord
			pubModality.publish(1)
			pubGoalPos.publish(msg)
			os.system('cls||clear') #clear the console
			print("\nModality 1 - Goal Reaching")
			print("(press '0' during the execution to cancel the target)")
			currentmode=1
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			

		elif command == 2: #second modality (Not Assisted Driving)
			
			currentmode=2
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			print("\nModality 2 - Not Assisted Driving\n")
			
			
		elif command == 3: #third modality (Assisted Driving)
			
			currentmode=3
			pubModality.publish(currentmode) #publish the value on 'mode' topic 
			print("\nModality 3 - Assisted Driving\n")
			
		elif command == 4:
			exit()
			
		else:
			print("Wrong key")


if __name__ == '__main__':
	main()

