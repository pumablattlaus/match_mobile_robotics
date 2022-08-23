#!/usr/bin/env python3
from ntpath import join
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

# moveit_commander.roscpp_initialize(sys.argv)



class JacobianPublisher():
    
    
    def __init__(self, ns='/mur216', planning_group='UR_arm'):
        rospy.init_node('jacobian_publisher_node', anonymous=True)
        self.config()
        self.joint_vel = Float64MultiArray()
        # robot = moveit_commander.RobotCommander()
        # scene = moveit_commander.PlanningSceneInterface()

        try:
            print("start")
            self.group = moveit_commander.MoveGroupCommander(planning_group, ns=ns, robot_description='mur216/robot_description')
        except Exception as e: 
            print(e)
            raise e
            
        self.pub = rospy.Publisher(ns+"/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        rospy.Subscriber(ns+"/joint_states",JointState, self.joint_state_cb)
        
        rospy.Subscriber(ns+"/arm_target_vel",Twist, self.target_vel_cb)
        # self.run()
        # rospy.spin()
        
            
    def joint_state_cb(self,joint_states):
    
        joint_states_array=[joint_states.position[0],joint_states.position[1],joint_states.position[2],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        J_ur=self.group.get_jacobian_matrix(joint_states_array)
        try:
            inverse = numpy.linalg.inv(J_ur)
        except numpy.linalg.LinAlgError as e:
            rospy.loginfo(e)
            rospy.loginfo(f"J_ur = {J_ur}")
            # inverse = numpy.identity(6)
            return
        target_dq = inverse.dot(self.target_vel)
        # rospy.loginfo(f"target_dq = {target_dq}")
        max_dq = abs(max(target_dq, key=abs))
        if max_dq > 0.1:
            for i in range(0,len(target_dq)):
                target_dq[i] = target_dq[i]/max_dq * 0.1
            rospy.loginfo(f"target_dq = {target_dq}")
            rospy.loginfo(f"restricted_dq = {target_dq}")

        
        self.joint_vel.data = target_dq
        self.pub.publish(self.joint_vel)
    
    def target_vel_cb(self,target_vel=Twist()):
        self.target_vel = numpy.array([target_vel.linear.x, target_vel.linear.y, target_vel.linear.z, target_vel.angular.x, target_vel.angular.y, target_vel.angular.z])

        
    def run(self):
        # rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        #     self.pub.publish(self.joint_vel)
            #pass 
            # print(self.joint_vel)
        rospy.spin()
        self.joint_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(self.joint_vel)

    def save_shutdown(self):
        # self.js_sub.unregister()
        rospy.loginfo("Save shutdown")
        joint_vel = Float64MultiArray()
        joint_vel.data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_vel=joint_vel
        self.pub.publish(self.joint_vel)
        self.pub.unregister() # no addional commands possible
        # rospy.signal_shutdown("save_shutdown")
        
    def config(self):
        self.target_vel = numpy.array([-0.0, -0.0, -0.0, 0.0, 0.0, 0.0])
        
    
    
if __name__ == '__main__':
    cart=JacobianPublisher()
    rospy.on_shutdown(cart.save_shutdown)
    rospy.spin()