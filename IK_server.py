#!/usr/bin/env python

### with this it has dropped in the bin.


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

def TF_Matrix(alpha,a,d,q):
	TF = Matrix([[ cos(q), -sin(q),0,a],
	     [sin(q)*cos(alpha),cos(q)*cos(alpha),-sin(alpha),-sin(alpha)*d],
	     [sin(q)*sin(alpha),cos(q)*sin(alpha),cos(alpha),cos(alpha)*d],
	     [0,0,0,1]])
	return TF

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


	##Homogenous Transforms		
    	T0_1 = TF_Matrix(alpha0,a0,d1,q1).subs(s)
    	T1_2 = TF_Matrix(alpha1,a1,d2,q2).subs(s)
    	T2_3 = TF_Matrix(alpha2,a2,d3,q3).subs(s)
        T3_4 = TF_Matrix(alpha3,a3,d4,q4).subs(s)
        T4_5 = TF_Matrix(alpha4,a4,d5,q5).subs(s)
	T5_6 = TF_Matrix(alpha5,a5,d6,q6).subs(s)
	T6_EE = TF_Matrix(alpha6,a6,d7,q7).subs(s)
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

	#Transformation from Base to Joint 3.
        R0_3 = simplify(T0_1 * T1_2 * T2_3)	

	#correction needed to account of orientation difference between definition of 
				#Gripper_link in URDF versus DH Convention
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
     
            ### Your IK code here 
	    
	    ###
	    # Calculate joint angles using Geometric IK method
	      #R0_6 = Rrpy * R_corr

 	    r_roll = rot_x(roll)
            r_pitch = rot_y(pitch)
            r_yaw = rot_z(yaw)
            R0_6= r_roll * r_pitch * r_yaw * R_corr  # Compensate for rotation discrepancy between DH parameters and Gazebo
           
	          
           
	    #Calculate Wrist Center:(WC)
            p_ee = Matrix([px,py,pz])
            WC = p_ee - R0_6 * Matrix([0, 0, s[d7]])
	    pprint(WC)

            theta1 = (atan2(WC[1],WC[0])).evalf() # @2 = atan2(y,x)
	    
	    s_a = 1.501
            s_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2)+pow((WC[2]-0.75),2))
            s_c = 1.25

	    angle_a = (acos((s_c**2 + s_b**2 - s_a**2)/(2*s_b*s_c))).evalf()
            angle_b = (acos((s_c**2 + s_a**2 - s_b**2)/(2*s_c*s_a))).evalf()
            angle_c = (acos((s_a**2 + s_b**2 - s_c**2)/(2*s_a*s_b))).evalf()

            theta2 = (pi/2 - angle_a - atan2(WC[2]-s[d1],sqrt(WC[0]**2 + WC[1]**2 - s[a1]))).evalf()
            theta3 = (pi/2 - (angle_b + 0.036)).evalf()

          
           
            R0_3 = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
            R0_3 = R0_3[0:3, 0:3]
	    pprint(R0_3)

	    #Euler Angles -- Inverse Kinematics lesson
            #R3_6 = R0_3.inv("LU")*R0_6
	    R3_6 = R0_3.T * R0_6


	    # Theta 4,5 & 6 angles.

            theta4 = (atan2(R3_6[2,2],-R3_6[0,2])).evalf()
            theta5 = (atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2),R3_6[1,2])).evalf()
            theta6 = (atan2(R3_6[1,1],R3_6[1,0])).evalf()
	
           
            print('theta1: ' + str(theta1))
	    print('theta2: ' + str(theta2))
	    print('theta3: ' + str(theta3))
	    print('theta4: ' + str(theta4))
	    print('theta5: ' + str(theta5))
	    print('theta6: ' + str(theta6))	
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
