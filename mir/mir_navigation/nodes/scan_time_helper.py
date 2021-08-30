#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

scan_frequency = 1.0
modified_scan_pub = None

def laser_scan_cb(msg):
    # type: (LaserScan) -> None
    modified_scan_msg = msg #type: (LaserScan)
    modified_scan_msg.time_increment = (1.0 / scan_frequency) / (len(modified_scan_msg.ranges) + 10)
    modified_scan_msg.scan_time = 1.0 / scan_frequency
    
    # rospy.loginfo("scan_time: " + str(modified_scan_msg.scan_time) + " | time_increment: " + str(modified_scan_msg.time_increment))

    modified_scan_pub.publish(modified_scan_msg)

if __name__ == '__main__':
    global modified_scan_pub
    global scan_frequency

    rospy.init_node("scan_time_helper")

    input_scan_topic = ""
    output_scan_topic = ""

    if rospy.has_param("~input_scan_topic"):
        input_scan_topic = rospy.get_param("~input_scan_topic")
    
    if rospy.has_param("~output_scan_topic"):
        output_scan_topic = rospy.get_param("~output_scan_topic")

    if rospy.has_param("~scan_frequency"):
        scan_frequency = rospy.get_param("~scan_frequency")

    rospy.loginfo("scan_helper node will use inputed scan info and add time information:")
    rospy.loginfo("Input Scan Topic: " + input_scan_topic)
    rospy.loginfo("Output Scan Topic: " + output_scan_topic)

    
    # move_base simple topic and action server
    scan_subscriber = rospy.Subscriber(input_scan_topic, LaserScan, laser_scan_cb)
    modified_scan_pub = rospy.Publisher(output_scan_topic, LaserScan, queue_size=10)

    rospy.spin()
