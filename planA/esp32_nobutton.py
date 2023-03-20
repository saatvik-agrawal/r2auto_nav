import rospy

#Set-up for IO pins based on our electronic architecture
push_button1 = "1"  # input as table 1
push_button2 = "2"  # input as table 2
push_button3 = "3"  # input as table 3
push_button4 = "4"  # input as table 4
push_button5 = "5"  # input as table 5
push_button6 = "6" # input as table 6
push_buttoncancel = "c"  # input as cancelOrder

def talker():
  command = input("Which table?\n");
	pub = rospy.Publisher('Payload', String, queue_size=10)
	pub.publish(command)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
