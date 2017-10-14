
## Project: Kinematics Pick & Place

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This program will try to pick and place the objects from the shelf and drop them in the bin.
This program uses the Kinematcis ( Forward and Inverse ) to calculate the positions and trajectory to drop in the bin.

[//]: # (Image References)

[image1]: ./images/Kuka-Arm1.PNG
[image2]: ./images/Kuka-Arm2.PNG
[image3]: ./images/Label-Kuka-Arm.jpg
[image4]: ./images/Theta-angles-1-2-3.PNG
[image5]: ./images/ik-copy.png
[image6]: ./images/l21-l-inverse-kinematics-01.png
[image7]: ./images/WC-coords.png
[image8]: ./images/l20-inverse-kinematics-02.png
[image9]: ./images/law-cosines-arctan.PNG
[image10]: ./images/law-cosines-1.PNG

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Used the below two images to create the kuka-arm in 2D by labelling the joints, links in the XYZ axis.

![KukaArm-Rzviz][image1]     ![KukaArm-Rzviz][image2] 


This is the Kuka-Arm with all the labels.

![Labelled-Kuka-Arm-2D][image3] 



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

DH Parameters Table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | 
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 
3->4 |  -pi/2 | -0.054 | 1.50 | 
4->5 | pi/2 | 0 | 0 | 
5->6 | -pi/2 | 0 | 0 | 
6->EE | 0 | 0 | 0.303 | 0

Calculated the homogenous transformations by substituting the DH parameters as 

      def TF_Matrix(alpha,a,d,q):
	TF = Matrix([[ cos(q), -sin(q),0,a],
	     [sin(q)*cos(alpha),cos(q)*cos(alpha),-sin(alpha),-sin(alpha)*d],
	     [sin(q)*sin(alpha),cos(q)*sin(alpha),cos(alpha),cos(alpha)*d],
	     [0,0,0,1]])
	return TF
	
 	T0_1 = TF_Matrix(alpha0,a0,d1,q1).subs(s)
    	T1_2 = TF_Matrix(alpha1,a1,d2,q2).subs(s)
    	T2_3 = TF_Matrix(alpha2,a2,d3,q3).subs(s)
        T3_4 = TF_Matrix(alpha3,a3,d4,q4).subs(s)
        T4_5 = TF_Matrix(alpha4,a4,d5,q5).subs(s)
	T5_6 = TF_Matrix(alpha5,a5,d6,q6).subs(s)
	T6_EE = TF_Matrix(alpha6,a6,d7,q7).subs(s)
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
      
    Then calculated the rotations as
      
      R_z = rot_z(radians(180))
      R_y = rot_y(radians(-90))
      R_corr= simplify(R_z * R_y)
        
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Calculation of the Theta angles as:


Used this diagram as reference to calculate the theta1 for the Kuka-Aram.
![theta1][image6]

As our kuka-arm has a spherical wrist, we take Zc as the wrist center and calculate the wrist center position and use them in the atan2 function to get the @1.

First calculated the Wrist Center Coordinates using this formula:
![Wrist Center Coords][image7]


Theta1:
     
     #Calculate WC and then calculate theta using atan2 function.
      
      p_ee = Matrix([px,py,pz])  # end effector position
      WC = p_ee - R0_6 * Matrix([0, 0, s[d7]]) # R0_6 is the rotation discrepancy between DH parameters and Gazebo
      theta1 = atan2(WC[1],WC[0]) # @1 = atan2(y,x)



Used this drawing to calculate the theta 2 & 3 angles.

![theta123-1][image5]

Used the Law of Cosines  (https://en.wikipedia.org/wiki/Law_of_cosines) to get the angles

![Cosine Law -1 ][image9]
![Cosine Law -ArcTan ][image10]


Theta2:
    
    	s_a = 1.501
        s_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2)+pow((WC[2]-0.75),2))
        s_c = 1.25

	angle_a = acos((s_c**2 + s_b**2 - s_a**2)/2*s_b*s_c)
        angle_b = acos((s_c**2 + s_a**2 - s_b**2)/2*s_c*s_a)
        angle_c = acos((s_a**2 + s_b**2 - s_c**2)/2*s_a*s_b)
        theta2 = pi/2 - angle_a - atan2(WC[2]-s[d1],sqrt(WC[0]**2 + WC[1]**2 - s[a1]))
 
 Theta3:
 
 	theta3 = pi/2 - (angle_b + 0.036)



  Theta4:
       
       	R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
        R3_6 = R0_3.inv("LU")*R0_6
       theta4 = atan2(R3_6[2,2],-R3_6[0,2])
  
  Theta5:
  
  	theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2),R3_6[1,2])
  
  Theta6:
  
  	theta6 = atan2(R3_6[1,1],R3_6[1,0])




### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Followed a step by step approach to build this project.
Initially created symbols and labelling the Kuka arm with joints, links, distances and etc.
Then created the DH Transformation matrix and using the FK, created the individual transformation matrix with symbols then subsituted the DH parameters
Calculated the rotation matrices and then the end-effector (wrist center) position and orientation from the  request.
Using the IK calculated the joint angles.

Need to improve the accuracy of picking and placing more items.

Please refer to the attached code in :  IK_server.py.

