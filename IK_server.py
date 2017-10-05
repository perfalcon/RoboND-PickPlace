#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def rot_x(gamma):
        R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(gamma),  -sin(gamma)],
                  [ 0,         sin(gamma),  cos(gamma)]])

        return R_x

def rot_y(beta):
        R_y = Matrix([[ cos(beta),        0,  sin(beta)],
                  [      0,        1,       0],
                  [-sin(beta),        0, cos(beta)]])

        return R_y

def rot_z(alpha):
        R_z = Matrix([[ cos(alpha),  -sin(alpha),       0],
                  [ sin(alpha),   cos(alpha),       0],
                  [      0,        0,       1]])

        return R_z


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
	#
	#   
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
	#
	#            
	# Define Modified DH Transformation matrix
	#i
	s = { alpha0:  0,     a0:0,    d1: 0.75,
                  alpha1: -pi/2,  a1:0.35,   d2: 0, q2: q2-pi/2,
                  alpha2:     0,  a2:1.25,   d3: 0,
                  alpha3: -pi/2,  a3:-0.054, d4: 1.50,
                  alpha4:  pi/2,  a4:0,      d5: 0,
                  alpha5: -pi/2,  a5:0,      d6: 0,
                  alpha6:  0,  a6:0,      d7: 0.303, q7: 0}

	#
	# Create individual transformation matrices
	#
	#
	T0_1 = Matrix([[ cos(q1), -sin(q1),0,a0],
                        [sin(q1)*cos(alpha0),cos(q1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1],
                        [sin(q1)*sin(alpha0),cos(q1)*sin(alpha0),cos(alpha0),cos(alpha0)*d1],
                        [0,0,0,1]])
        T0_1=T0_1.subs(s)
        T1_2 = Matrix([[ cos(q2), -sin(q2),0,a0],
                        [sin(q2)*cos(alpha1),cos(q2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2],
                        [sin(q2)*sin(alpha1),cos(q2)*sin(alpha1),cos(alpha1),cos(alpha1)*d2],
                        [0,0,0,1]])
        T1_2=T1_2.subs(s)
        T2_3 = Matrix([[ cos(q3), -sin(q3),0,a0],
                                                                [sin(q3)*cos(alpha2),cos(q3)*cos(alpha2),-sin(alpha2),-sin(alpha2)*d3],
                                                                [sin(q3)*sin(alpha2),cos(q3)*sin(alpha2),cos(alpha2),cos(alpha2)*d3],
                                                                [0,0,0,1]])
        T2_3=T2_3.subs(s)
        T3_4 = Matrix([[ cos(q4), -sin(q4),0,a0],
                                                                [sin(q4)*cos(alpha3),cos(q4)*cos(alpha3),-sin(alpha3),-sin(alpha3)*d4],
                                                                [sin(q4)*sin(alpha3),cos(q4)*sin(alpha3),cos(alpha3),cos(alpha3)*d4],
                                                                [0,0,0,1]])
        T3_4=T3_4.subs(s)
        T4_5 = Matrix([[ cos(q5), -sin(q5),0,a0],
                                                                [sin(q5)*cos(alpha4),cos(q5)*cos(alpha4),-sin(alpha4),-sin(alpha4)*d5],
                                                                [sin(q5)*sin(alpha4),cos(q5)*sin(alpha4),cos(alpha4),cos(alpha4)*d5],
                                                                [0,0,0,1]])
        T4_5=T4_5.subs(s)
        T5_6 = Matrix([[ cos(q6), -sin(q6),0,a0],
                        [sin(q6)*cos(alpha5),cos(q6)*cos(alpha5),-sin(alpha5),-sin(alpha5)*d6],
                                                                [sin(q6)*sin(alpha5),cos(q6)*sin(alpha5),cos(alpha5),cos(alpha5)*d6],
                                                                [0,0,0,1]])
        T5_6=T5_6.subs(s)
        T6_G = Matrix([[ cos(q7), -sin(q7),0,a0],
                                                                [sin(q7)*cos(alpha6),cos(q7)*cos(alpha6),-sin(alpha6),-sin(alpha6)*d7],
                                                                [sin(q7)*sin(alpha6),cos(q7)*sin(alpha6),cos(alpha6),cos(alpha6)*d7],
                                                                [0,0,0,1]])
        T6_G=T6_G.subs(s)

	
	#T0_2=simplify(T0_1*T1_2)
        #T0_3=simplify(T0_2*T2_3)
        #T0_4=simplify(T0_3*T3_4)
        #T0_5=simplify(T0_4*T4_5)
        #T0_6=simplify(T0_5*T5_6)
        #T0_G=simplify(T0_6*T6_G)

	# Extract rotation matrices from the transformation matrices
	
	    R_z = rot_z(radians(180))
        R_y = rot_y(radians(-90))

        R_corr= simplify(R_z * R_y)

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
	    print "x-->",x
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
        
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	        
            r_roll = rot_x(roll)
            r_pitch = rot_y(pitch)
            r_yaw = rot_z(yaw)
            R0_6= r_roll * r_pitch * r_yaw * R_corr
        #Calculate WC:
        
			p_ee = Matrix([px,py,pz])
            WC = p_ee - R0_6 * Matrix([0, 0, s[d7]])

		# Calculate joint angles using Geometric IK method	
			
            theta1 = atan2(WC[1],WC[0]) # @2 = atan2(y,x)
	    
	        s_a = 1.501
            s_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2)+pow((WC[2]-0.75),2))
            s_c = 1.25

	        angle_a = acos((s_c**2 + s_b**2 - s_a**2)/2*s_b*s_c)
            angle_b = acos((s_c**2 + s_a**2 - s_b**2)/2*s_c*s_a)
            angle_c = acos((s_a**2 + s_b**2 - s_c**2)/2*s_a*s_b)

            theta2 = pi/2 - angle_a - atan2(WC[2]-s[d1],sqrt(WC[0]**2 + WC[1]**2 - s[a1]))
            theta3 = pi/2 - (angle_b + 0.036)

            R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
            R3_6 = R0_3.inv("LU")*R0_6

            theta4 = atan2(R3_6[2,2],-R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2),R3_6[1,2])
            theta6 = atan2(R3_6[1,1],R3_6[1,0])
	
            print "theta1, theta2, theta3, theta4, theta5, theta6",theta1,theta2,theta3,theta3,theta4,theta5,theta6 				
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
