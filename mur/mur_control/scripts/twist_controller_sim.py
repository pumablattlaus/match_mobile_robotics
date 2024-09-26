#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from math import sin, cos, pi
import moveit_commander


class TwistControllerSim():

    def config(self):
        self.UR_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.UR_prefix = rospy.get_param('~UR_prefix', 'UR10_l/')
        self.rate = rospy.get_param('~rate', 100)
        self.robot_name = rospy.get_param('~robot_name', 'mur620a')


    def __init__(self) -> None:
        self.config()
        # Define the DH parameters for the UR10e robot
        self.a = [0, -0.612, -0.5723, 0, 0, 0]
        self.alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        self.d = [ 0.1807, 0, 0, 0.17415, 0.11985,  0.11655]
        #self.d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]
        self.theta = [0, 0, 0, 0, 0, 0]


        # Initialize the moveit_commander module
        moveit_commander.roscpp_initialize([])
        robot = moveit_commander.RobotCommander(robot_description=self.robot_name + '/robot_description', ns=self.robot_name)
        self.group = moveit_commander.MoveGroupCommander("UR_arm_l" , robot_description=self.robot_name + '/robot_description', ns=self.robot_name)

        # subscribe to joint states
        joint_state_topic = rospy.get_param('~joint_state_topic', '/mur620a/joint_states')
        rospy.Subscriber(joint_state_topic, JointState, self.joint_state_callback)

        rospy.wait_for_message(joint_state_topic, JointState)
        rospy.loginfo('Received joint states')

        # initialize the publisher for the joint velocities
        group_vel_controller_topic = rospy.get_param('~group_vel_controller_topic', '/mur620a/joint_group_vel_controller_l/unsafe/command')
        self.group_vel_controller_pub = rospy.Publisher(group_vel_controller_topic, Float64MultiArray, queue_size=1)




    # Compute the transformation matrix from the base frame to the end-effector frame
    def compute_jacobian(self,theta):
        a1 = self.a[1]
        a2 = self.a[2]
        d1 = self.d[0]
        d4 = self.d[3]
        d3 = 0
        d5 = self.d[4]
        alpha0 = self.alpha[0]
        alpha3 = self.alpha[3]
        alpha4 = self.alpha[4]
        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]
        q4 = theta[3]
        q5 = theta[4]
        q6 = theta[5]

        J = ([[-a1*sin(q1)*cos(q2) - a1*sin(q2)*cos(alpha0)*cos(q1) + a2*(sin(q1)*sin(q2) - cos(alpha0)*cos(q1)*cos(q2))*sin(q3) - a2*(sin(q1)*cos(q2) + sin(q2)*cos(alpha0)*cos(q1))*cos(q3) + d3*sin(alpha0)*cos(q1) + d4*(sin(alpha0)*cos(alpha3)*cos(q1) - sin(alpha3)*sin(q1)*sin(q2 + q3 + q4) + sin(alpha3)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4)) - d5*((sin(q1)*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*cos(alpha0)*cos(q1))*sin(alpha4)*sin(q5) + (sin(alpha0)*sin(alpha3)*cos(q1) + sin(q1)*sin(q2 + q3 + q4)*cos(alpha3) - cos(alpha0)*cos(alpha3)*cos(q1)*cos(q2 + q3 + q4))*sin(alpha4)*cos(q5) - (sin(alpha0)*cos(alpha3)*cos(q1) - sin(alpha3)*sin(q1)*sin(q2 + q3 + q4) + sin(alpha3)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4))*cos(alpha4)), -a1*sin(q1)*cos(alpha0)*cos(q2) - a1*sin(q2)*cos(q1) - a2*sin(q1)*cos(alpha0)*cos(q2 + q3) - a2*sin(q2 + q3)*cos(q1) - d4*sin(alpha3)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0) + d4*sin(alpha3)*cos(q1)*cos(q2 + q3 + q4) - d5*sin(alpha3)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha4) + d5*sin(alpha3)*cos(alpha4)*cos(q1)*cos(q2 + q3 + q4) - d5*sin(alpha4)*sin(q1)*sin(q5)*cos(alpha0)*cos(q2 + q3 + q4) - d5*sin(alpha4)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha3)*cos(q5) - d5*sin(alpha4)*sin(q5)*sin(q2 + q3 + q4)*cos(q1) + d5*sin(alpha4)*cos(alpha3)*cos(q1)*cos(q5)*cos(q2 + q3 + q4), -a2*sin(q1)*cos(alpha0)*cos(q2 + q3) - a2*sin(q2 + q3)*cos(q1) - d4*sin(alpha3)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0) + d4*sin(alpha3)*cos(q1)*cos(q2 + q3 + q4) - d5*sin(alpha3)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha4) + d5*sin(alpha3)*cos(alpha4)*cos(q1)*cos(q2 + q3 + q4) - d5*sin(alpha4)*sin(q1)*sin(q5)*cos(alpha0)*cos(q2 + q3 + q4) - d5*sin(alpha4)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha3)*cos(q5) - d5*sin(alpha4)*sin(q5)*sin(q2 + q3 + q4)*cos(q1) + d5*sin(alpha4)*cos(alpha3)*cos(q1)*cos(q5)*cos(q2 + q3 + q4), -d4*sin(alpha3)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0) + d4*sin(alpha3)*cos(q1)*cos(q2 + q3 + q4) - d5*sin(alpha3)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha4) + d5*sin(alpha3)*cos(alpha4)*cos(q1)*cos(q2 + q3 + q4) - d5*sin(alpha4)*sin(q1)*sin(q5)*cos(alpha0)*cos(q2 + q3 + q4) - d5*sin(alpha4)*sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha3)*cos(q5) - d5*sin(alpha4)*sin(q5)*sin(q2 + q3 + q4)*cos(q1) + d5*sin(alpha4)*cos(alpha3)*cos(q1)*cos(q5)*cos(q2 + q3 + q4), d5*(sin(alpha0)*sin(alpha3)*sin(q1)*sin(q5) - sin(q1)*sin(q5)*cos(alpha0)*cos(alpha3)*cos(q2 + q3 + q4) - sin(q1)*sin(q2 + q3 + q4)*cos(alpha0)*cos(q5) - sin(q5)*sin(q2 + q3 + q4)*cos(alpha3)*cos(q1) + cos(q1)*cos(q5)*cos(q2 + q3 + q4))*sin(alpha4), 0], [-a1*sin(q1)*sin(q2)*cos(alpha0) + a1*cos(q1)*cos(q2) - a2*(sin(q1)*sin(q2)*cos(alpha0) - cos(q1)*cos(q2))*cos(q3) - a2*(sin(q1)*cos(alpha0)*cos(q2) + sin(q2)*cos(q1))*sin(q3) + d3*sin(alpha0)*sin(q1) + d4*(sin(alpha0)*sin(q1)*cos(alpha3) + sin(alpha3)*sin(q1)*cos(alpha0)*cos(q2 + q3 + q4) + sin(alpha3)*sin(q2 + q3 + q4)*cos(q1)) + d5*(-(sin(q1)*sin(q2 + q3 + q4)*cos(alpha0) - cos(q1)*cos(q2 + q3 + q4))*sin(alpha4)*sin(q5) - (sin(alpha0)*sin(alpha3)*sin(q1) - sin(q1)*cos(alpha0)*cos(alpha3)*cos(q2 + q3 + q4) - sin(q2 + q3 + q4)*cos(alpha3)*cos(q1))*sin(alpha4)*cos(q5) + (sin(alpha0)*sin(q1)*cos(alpha3) + sin(alpha3)*sin(q1)*cos(alpha0)*cos(q2 + q3 + q4) + sin(alpha3)*sin(q2 + q3 + q4)*cos(q1))*cos(alpha4)), -a1*sin(q1)*sin(q2) + a1*cos(alpha0)*cos(q1)*cos(q2) - a2*sin(q1)*sin(q2 + q3) + a2*cos(alpha0)*cos(q1)*cos(q2 + q3) + d4*sin(alpha3)*sin(q1)*cos(q2 + q3 + q4) + d4*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha0)*cos(q1) + d5*sin(alpha3)*sin(q1)*cos(alpha4)*cos(q2 + q3 + q4) + d5*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha4)*cos(q1) - d5*sin(alpha4)*sin(q1)*sin(q5)*sin(q2 + q3 + q4) + d5*sin(alpha4)*sin(q1)*cos(alpha3)*cos(q5)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q5)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha3)*cos(q1)*cos(q5), -a2*sin(q1)*sin(q2 + q3) + a2*cos(alpha0)*cos(q1)*cos(q2 + q3) + d4*sin(alpha3)*sin(q1)*cos(q2 + q3 + q4) + d4*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha0)*cos(q1) + d5*sin(alpha3)*sin(q1)*cos(alpha4)*cos(q2 + q3 + q4) + d5*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha4)*cos(q1) - d5*sin(alpha4)*sin(q1)*sin(q5)*sin(q2 + q3 + q4) + d5*sin(alpha4)*sin(q1)*cos(alpha3)*cos(q5)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q5)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha3)*cos(q1)*cos(q5), d4*sin(alpha3)*sin(q1)*cos(q2 + q3 + q4) + d4*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha0)*cos(q1) + d5*sin(alpha3)*sin(q1)*cos(alpha4)*cos(q2 + q3 + q4) + d5*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha4)*cos(q1) - d5*sin(alpha4)*sin(q1)*sin(q5)*sin(q2 + q3 + q4) + d5*sin(alpha4)*sin(q1)*cos(alpha3)*cos(q5)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q5)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q2 + q3 + q4)*cos(alpha0)*cos(alpha3)*cos(q1)*cos(q5), d5*(-sin(alpha0)*sin(alpha3)*sin(q5)*cos(q1) - sin(q1)*sin(q5)*sin(q2 + q3 + q4)*cos(alpha3) + sin(q1)*cos(q5)*cos(q2 + q3 + q4) + sin(q5)*cos(alpha0)*cos(alpha3)*cos(q1)*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*cos(alpha0)*cos(q1)*cos(q5))*sin(alpha4), 0], [0, (a1*cos(q2) + a2*cos(q2 + q3) + d4*sin(alpha3)*sin(q2 + q3 + q4) + d5*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha4) + d5*sin(alpha4)*sin(q5)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q2 + q3 + q4)*cos(alpha3)*cos(q5))*sin(alpha0), (a2*cos(q2 + q3) + d4*sin(alpha3)*sin(q2 + q3 + q4) + d5*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha4) + d5*sin(alpha4)*sin(q5)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q2 + q3 + q4)*cos(alpha3)*cos(q5))*sin(alpha0), (d4*sin(alpha3)*sin(q2 + q3 + q4) + d5*sin(alpha3)*sin(q2 + q3 + q4)*cos(alpha4) + d5*sin(alpha4)*sin(q5)*cos(q2 + q3 + q4) + d5*sin(alpha4)*sin(q2 + q3 + q4)*cos(alpha3)*cos(q5))*sin(alpha0), d5*(sin(alpha0)*sin(q5)*cos(alpha3)*cos(q2 + q3 + q4) + sin(alpha0)*sin(q2 + q3 + q4)*cos(q5) + sin(alpha3)*sin(q5)*cos(alpha0))*sin(alpha4), 0], [0, sin(alpha0)*sin(q1), sin(alpha0)*sin(q1), sin(alpha0)*sin(q1), sin(alpha0)*sin(q1)*cos(alpha3) + sin(alpha3)*sin(q1)*cos(alpha0)*cos(q2 + q3 + q4) + sin(alpha3)*sin(q2 + q3 + q4)*cos(q1), -(sin(q1)*sin(q2 + q3 + q4)*cos(alpha0) - cos(q1)*cos(q2 + q3 + q4))*sin(alpha4)*sin(q5) + (-sin(alpha0)*sin(alpha3)*sin(q1) + sin(q1)*cos(alpha0)*cos(alpha3)*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*cos(alpha3)*cos(q1))*sin(alpha4)*cos(q5) + (sin(alpha0)*sin(q1)*cos(alpha3) + sin(alpha3)*sin(q1)*cos(alpha0)*cos(q2 + q3 + q4) + sin(alpha3)*sin(q2 + q3 + q4)*cos(q1))*cos(alpha4)], [0, -sin(alpha0)*cos(q1), -sin(alpha0)*cos(q1), -sin(alpha0)*cos(q1), -sin(alpha0)*cos(alpha3)*cos(q1) + sin(alpha3)*sin(q1)*sin(q2 + q3 + q4) - sin(alpha3)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4), (sin(q1)*cos(q2 + q3 + q4) + sin(q2 + q3 + q4)*cos(alpha0)*cos(q1))*sin(alpha4)*sin(q5) + (sin(alpha0)*sin(alpha3)*cos(q1) + sin(q1)*sin(q2 + q3 + q4)*cos(alpha3) - cos(alpha0)*cos(alpha3)*cos(q1)*cos(q2 + q3 + q4))*sin(alpha4)*cos(q5) - (sin(alpha0)*cos(alpha3)*cos(q1) - sin(alpha3)*sin(q1)*sin(q2 + q3 + q4) + sin(alpha3)*cos(alpha0)*cos(q1)*cos(q2 + q3 + q4))*cos(alpha4)], [1, cos(alpha0), cos(alpha0), cos(alpha0), -sin(alpha0)*sin(alpha3)*cos(q2 + q3 + q4) + cos(alpha0)*cos(alpha3), (-sin(alpha0)*sin(alpha3)*cos(q2 + q3 + q4) + cos(alpha0)*cos(alpha3))*cos(alpha4) - (sin(alpha0)*cos(alpha3)*cos(q2 + q3 + q4) + sin(alpha3)*cos(alpha0))*sin(alpha4)*cos(q5) + sin(alpha0)*sin(alpha4)*sin(q5)*sin(q2 + q3 + q4)]])
        #print(J)

        # get joint values from move_group
        joint_values = self.group.get_current_joint_values()
        print(joint_values)
        print(self.joint_states)
        # get jacobain matrix from move_group
        
        J = self.group.get_jacobian_matrix(joint_values)
        #print(J)

        return J
    

    # Compute the inverse differential kinematics using the Jacobian matrix
    def compute_inverse_differential_kinematics(self,jacobian, desired_twist):
        joint_velocities = np.linalg.pinv(jacobian) @ desired_twist

        return joint_velocities


    def joint_state_callback(self, msg):
        # get the correct joint states by name
        self.joint_states = []
        for joint_name in self.UR_joint_names:
            self.joint_states.append(msg.position[msg.name.index(self.UR_prefix + joint_name)])
        

    def run(self):
        desired_twist = np.array([-0.0, -0.0, -0.05, 0, 0, 0])  # Replace with the desired twist

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            jacobian = self.compute_jacobian(self.joint_states)
            joint_velocities = self.compute_inverse_differential_kinematics(jacobian, desired_twist)

            # publish the joint velocities
            joint_velocities_msg = Float64MultiArray()
            joint_velocities_msg.data = joint_velocities
            #print(joint_velocities)
            self.group_vel_controller_pub.publish(joint_velocities_msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('twist_controller_sim')
    twist_controller = TwistControllerSim()
    rospy.sleep(1)
    twist_controller.run()