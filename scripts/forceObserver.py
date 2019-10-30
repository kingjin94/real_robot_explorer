#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Vector3
import numpy as np
import tf
from copy import deepcopy
from gazebo_msgs.msg import ContactsState, ContactState

ABS_FORCE_THRES = 5.0

class ForceObserver:
	def __init__(self, topic):
		self.tf_listener = tf.TransformListener()
		self.force_sub = rospy.Subscriber(topic, WrenchStamped, callback=self.force_listener)
		self.coll_pub = rospy.Publisher("/panda/bumper/panda_probe_ball", ContactsState)

	def threeDVecMsg2Numpy(self, msg):
		return np.asarray((msg.x, msg.y, msg.z))

	def force_listener(self, msg):
		#print(msg)
		force = self.threeDVecMsg2Numpy(msg.wrench.force)
		#print(force)
		tool_weight_msg = Vector3Stamped()
		tool_weight_msg.header = deepcopy(msg.header)
		tool_weight_msg.header.frame_id = "world"
		tool_weight_msg.vector.z = 1.0 # about 100 g...
		start = rospy.Time.now()
		while not self.tf_listener.canTransform("world", msg.header.frame_id, msg.header.stamp):
			rospy.sleep(0.001) # wait for sync in tf
			if rospy.Time.now() > start + rospy.Duration.from_sec(0.1):
				print("No Transform")
				return
		tool_weight_msg=self.tf_listener.transformVector3(msg.header.frame_id, tool_weight_msg)
		#print(tool_weight_msg)
		tool_weight_force = self.threeDVecMsg2Numpy(tool_weight_msg.vector)
		net_force = force - tool_weight_force
		if np.linalg.norm(net_force) < ABS_FORCE_THRES:
			return
		# TODO: Filter to high accelertations of eef
		normal = net_force/np.linalg.norm(net_force)
		c_point = normal * (0.015)
		print(c_point)
		ret_msg = ContactsState()
		ret_msg.header = deepcopy(msg.header)
		contact_state = ContactState()
		contact_state.contact_positions.append(Vector3(*c_point))
		ret_msg.states.append(contact_state)
		self.coll_pub.publish(ret_msg)

if __name__ == '__main__':	
	rospy.init_node("FExt_To_Collision")
	
	Obs = ForceObserver("/franka_state_controller/F_ext")
	
	rospy.spin()
