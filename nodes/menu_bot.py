import rospy, socket
from std_msgs.msg import String

'''
This module implements a barebones menu bot for anon_node_launcher
'''
PORT = 39525
IP = '127.0.0.1'

class Menu_Bot(object):
	'''
	'''

	def __init__(self):
		'''
		'''
		rospy.init_node("menu_bot")

		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		while not rospy.is_shutdown():
			# print menu options
			self.print_launch_menu()
			garbage = raw_input(">> ")
			self.sock.sendto(str(garbage), (IP, PORT))
			if (garbage.lower() == "q"):
				print("exiting...")
				exit()
		

	def print_launch_menu(self):
		'''
		This functin prints out terminal menu for launching nodes
		'''
		print("==============================")
		print("Menu:")
		print(" - Enter '1' to spawn node_A")
		print(" - Enter '2' to spawn node_B")
		print(" - Enter '3' to spawn node_C")
		print(" - Enter 'q' to exit")
		print("==============================")


if __name__ == "__main__":
	bot = Menu_Bot()