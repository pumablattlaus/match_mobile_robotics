#!/usr/bin/env python3 

# this node is used to publish a fake localization for all robots using the ground truth pose data
import rospy

import tf
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped

class FakeLocalization():

    def __init__(self):
        rospy.init_node('fake_localization_handler')
        self.pose_old = Odometry()
        self.time_stamp_old = rospy.Time.now()
        self.redundant_timestamp_index = 0
        self.tf_prefix_slashed = rospy.get_param('~tf_prefix_slashed',"mur216/")
        self.frame_id = rospy.get_param('~frame_id',"base_footprint")
        rospy.Subscriber('/%sground_truth' % self.tf_prefix_slashed, Odometry, self.fake_localization_handler)
        # self.fake_amcl_pub = rospy.Publisher('amcl_pose' , PoseWithCovarianceStamped, queue_size=1)
        rospy.spin()

    def fake_localization_handler(self,msg):

        t = rospy.Time.now()
        br = tf.TransformBroadcaster()
        if (msg.pose.pose.position.x == self.pose_old.pose.pose.position.x and msg.pose.pose.position.y == self.pose_old.pose.pose.position.y and msg.pose.pose.orientation.x == self.pose_old.pose.pose.orientation.x 
        and msg.pose.pose.orientation.y == self.pose_old.pose.pose.orientation.y and msg.pose.pose.orientation.z == self.pose_old.pose.pose.orientation.z and msg.pose.pose.orientation.w == self.pose_old.pose.pose.orientation.w):
            pass
        elif abs(t-self.time_stamp_old)> rospy.Duration(0.001):
            # publish fake localization
            self.redundant_timestamp_index = 0
            br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,0), # publish the footprint of the robot
                            (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                            t,
                            self.tf_prefix_slashed +  self.frame_id,
                            "map")
        else:
            self.redundant_timestamp_index += 1
            br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,0), # publish the footprint of the robot
                            (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                            t + rospy.Duration(0.0001)*self.redundant_timestamp_index, # avoid redundant timestamp
                            self.tf_prefix_slashed  + self.frame_id,
                            "map")
        self.time_stamp_old = t

if __name__ == '__main__':
    FakeLocalization()
