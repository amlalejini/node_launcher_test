#!/usr/bin/env python

import rospy, time, atexit, signal
from std_msgs.msg import String

MSG_PARAM = "msg"
ANON_NAME_TOPIC = "anon_name_report"


'''
This module contains a dummy anonymous node that publishes string messages over a chatter topic.
'''

class Dummy_Node_B(object):
	'''
	'''
	def __init__(self):
		'''
		'''
		# initialize as anonymous node
		rospy.init_node("anon_node_B", anonymous = True)
	
		# register exit handler
		atexit.register(self.exit_handler)
		# register SIGINT handler
		signal.signal(signal.SIGINT, self.signal_handler)

		# create dummy chatter node for testing purposes
		pub = rospy.Publisher(rospy.get_name() + "/type_B_chatter", String)
		# initialize anonymous name publisher
		name_pub = rospy.Publisher(ANON_NAME_TOPIC, String)
		time.sleep(0.25)
		# create standard dummy message
		my_msg = rospy.get_param("/" + rospy.get_name() + "/" + str(MSG_PARAM))
		# run!
		
		# publish anonymous name so node launcher can see
		stringy = "%s" % str(rospy.get_name())
		name_pub.publish(stringy)

		while not rospy.is_shutdown():
			stringy = "Chatter: %s \n@ %s" % (my_msg, rospy.get_time())
			pub.publish(stringy)
			time.sleep(1)
	
	def signal_handler(self, signal, frame):
		'''
		'''
		exit()

	def exit_handler(self):
		'''
		'''
		rospy.delete_param("/" + str(rospy.get_name()) + "/" + str(MSG_PARAM))
		print("params cleaned up!")
		exit()

if __name__ == "__main__":
	test = Dummy_Node_B()