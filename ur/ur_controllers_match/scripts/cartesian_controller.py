#!/usr/bin/env python3
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
            
        self.pub = rospy.Publisher(ns+"/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        rospy.Subscriber(ns+"/joint_states",JointState, self.joint_state_cb)
        rospy.Subscriber(ns+"/arm_target_vel",Twist, self.target_vel_cb)
        self.run()
        rospy.spin()
        
            
    def joint_state_cb(self,joint_states):
    
        joint_states_array=[joint_states.position[0],joint_states.position[1],joint_states.position[2],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        J_ur=self.group.get_jacobian_matrix(joint_states_array)
        inverse = numpy.linalg.inv(J_ur) #TODO: try except
        target_dq = inverse.dot(self.target_vel)
        
        for i in range(0,len(target_dq)):
            if abs(target_dq[i]) > 0.1:
                target_dq[i] = target_dq[i]/abs(target_dq[i]) * 0.1 
        
        self.joint_vel.data = target_dq
    
    def target_vel_cb(self,target_vel=Twist()):
        self.target_vel = numpy.array([target_vel.linear.x, target_vel.linear.y, target_vel.linear.z, target_vel.angular.x, target_vel.angular.y, target_vel.angular.z])

        
    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.pub.publish(self.joint_vel)
            #pass 
            # print(self.joint_vel)
            
        self.joint_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(self.joint_vel)
        
    def config(self):
        self.target_vel = numpy.array([-0.0, -0.0, -0.0, 0.0, 0.0, 0.0])
        
    
    
if __name__ == '__main__':
    JacobianPublisher()