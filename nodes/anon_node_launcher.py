import rospy, os, atexit, signal, socket, rosnode

from std_msgs.msg import String
from multiprocessing import Process

'''
This module provides an interface to launch anonymous nodes.
'''
QUIT_MSG = "q"
LAUNCH_A = "1"
LAUNCH_B = "2"
LAUNCH_C = "3"
ANON_NAME_TOPIC = "anon_name_report"

PORT = 39525
IP = '127.0.0.1'

BUFFER_SIZE = 1024

class Node_Launcher(object):
	'''
	'''
	def __init__(self):
		'''
		'''
		rospy.init_node("node_launcher")
		rospy.Subscriber(ANON_NAME_TOPIC, String, self.anon_name_callback)

		self.process_list = []					# maintains list of all processes
		self.anon_node_list = []				# maintains list of all child anonymous node names
		atexit.register(self.exit_handler)
		signal.signal(signal.SIGINT, self.signal_handler)

		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind((IP, PORT))
		print("Listening for input on port %d..." % PORT)
		self.run_launcher()

	def anon_name_callback(self, data):
		'''
		This is the callback function for the /anon_name_report topic.
		- When anonymous nodes are spawned from this module, they will report their name
			back to this node via this topic.
		- This callback function appends received names to child node name list
		'''
		node_name = str(data.data)
		print("Received name: " + str(node_name))
		self.anon_node_list.append(node_name)

	def run_launcher(self):
		'''
		This function is the callback function for the launcher menu topic.
		- Waits for user input to determine which anonymous node it will spawn.
		- Upon receiving valid input, spawn anonymous node.
		'''
		while not rospy.is_shutdown():
			data, addr = self.sock.recvfrom(BUFFER_SIZE)
			menu_msg = str(data).lower()
			print ("Menu selection: %s" % menu_msg)
			fname = None 
			msg = None
			if (menu_msg == QUIT_MSG):
				print("exiting...")
				exit()
			elif (menu_msg == LAUNCH_A):
				print("Launching anon node A...")
				fname = "anon_node_A.py"
				msg = "A"
			elif (menu_msg == LAUNCH_B):
				print("Launching anon node B...")
				fname = "anon_node_B.py"
				msg = "B"
			elif (menu_msg == LAUNCH_C):
				print("Launching anon node C...")
				fname = "anon_node_C.py"
				msg = "C"
			else:
				continue

			garbo_process = Process(target = self.launch_node, args = (fname, msg))
			garbo_process.start()
			self.process_list.append(garbo_process)
			print("Node launched!")

	def launch_node(self, fname, msg):
		'''
		'''
		stringy = "rosrun node_launcher_test %s _msg:=%s" % (fname, msg)
		os.system(stringy)

	def signal_handler(self, signal, frame):
		'''
		'''
		exit()

	def exit_handler(self):
		'''
		This function is called on exit.
		- Cleans up all child processes
		'''
		child_pids = [str(p.pid) for p in self.process_list]

		# Kill all child nodes
		print("Nodes to kill: " + str(self.anon_node_list))
		rosnode.kill_nodes(self.anon_node_list)

		# Join all child processes
		print("Children to be destroyed: %s" % (child_pids))
		for p in self.process_list:
		 	print("Trying to destroy %s" % (str(p.pid)))
		 	p.join()
		 	print("cleaned up process %s" % (str(p.pid)))

if __name__ == "__main__":
	thing = Node_Launcher()
