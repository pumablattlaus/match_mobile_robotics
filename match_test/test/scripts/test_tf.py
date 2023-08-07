#! /usr/bin/env python3
import unittest
import rostest
import rospy
import tf

# class ParamsTestCase(unittest.TestCase):
# 	def test_param(self):
# 		self.assertEqual(rospy.get_param('/value'), 10)

# 		# value = rospy.get_param('/value', None)
# 		# self.assertIsNotNone(value)

class TfTestCase(unittest.TestCase):
	def test_tf(self):
		rospy.init_node('test_tf')
		tf_listener = tf.TransformListener()
		# ggf mit ns:
		child = rospy.get_param("test_link_child")
		parent = rospy.get_param("test_link_parent", "map")
		tf_listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(30.0))
		# self.assertTrue(tf_listener.frameExists('/base_link'))

		# test if transform to map for all frames exist
		frames = tf_listener.getFrameStrings()
		for frame in frames:
			try:
				tf_listener.waitForTransform(parent, frame, rospy.Time(0), rospy.Duration(5.0))
			except:
				self.fail('Transform to map for frame %s does not exist' % frame)



if __name__ == '__main__':
	rostest.rosrun('match_test', 'test_tf', TfTestCase)
