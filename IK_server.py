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
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6=symbols('alpha0:7')
            a0,a1,a2,a3,a4,a5,a6=symbols('a0:7')
            # Joint angle symbols
            q1,q2,q3,q4,q5,q6,q7= symbols('q1:8')
            d1,d2,d3,d4,d5,d6,d7= symbols('d1:8') 
            # Modified DH params
            #Defining constants
            S={alpha0:0,a0:0,d1:0.75,
               alpha1:-pi/2,a1:0.35,d2:0,q2:q2-pi/2,
               alpha2:0,a2:1.25,d3:0,
               alpha3:-pi/2,a3:-0.054,d4:1.5,
               alpha4:pi/2,a4:0,d5:0,
               alpha5:-pi/2,a5:0,d6:0,
               alpha6:0,a6:0,d7:0.303,q7:0}
            
            # Define Modified DH Transformation matrix
         
             

            # Create individual transformation matrices
            T0_1=Matrix([[cos(q1),-sin(q1),0,a0],
                         [sin(q1)*cos(alpha0),cos(q1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1],
                         [sin(q1)*sin(alpha0),cos(q1)*sin(alpha0),cos(alpha0),cos(alpha0)*d1],
                          [0,0,0,1]])
            T0_1=T0_1.subs(S)

            T1_2=Matrix([[cos(q2),-sin(q2),0,a1],
                         [sin(q2)*cos(alpha1),cos(q2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2],
                         [sin(q2)*sin(alpha1),cos(q2)*sin(alpha1),cos(alpha1),cos(alpha1)*d2],
                         [0,0,0,1]])
            T1_2=T1_2.subs(S)

            T2_3=Matrix([[cos(q3),-sin(q3),0,a2],
                         [sin(q3)*cos(alpha2),cos(q3)*cos(alpha2),-sin(alpha2),-sin(alpha2)*d3],
                         [sin(q3)*sin(alpha2),cos(q3)*sin(alpha2),cos(alpha2),cos(alpha2)*d3],
                         [0,0,0,1]])

            T2_3=T2_3.subs(S)
            T3_4=Matrix([[cos(q4),-sin(q4),0,a3],
                         [sin(q4)*cos(alpha3),cos(q4)*cos(alpha3),-sin(alpha3),-sin(alpha3)*d4],
                         [sin(q4)*sin(alpha3),cos(q4)*sin(alpha3),cos(alpha3),cos(alpha3)*d4],
                         [0,0,0,1]])
            T3_4=T3_4.subs(S)
            T4_5= Matrix([[cos(q5),-sin(q5),0,a4],
                         [sin(q5)*cos(alpha4),cos(q5)*cos(alpha4),-sin(alpha4),-sin(alpha4)*d5],
                         [sin(q5)*sin(alpha4),cos(q5)*sin(alpha4),cos(alpha4),cos(alpha4)*d5],
                         [0,0,0,1]])


            T4_5=T4_5.subs(S)
            T5_6= Matrix([[cos(q6),-sin(q6),0,a5],
                         [sin(q6)*cos(alpha5),cos(q6)*cos(alpha5),-sin(alpha5),-sin(alpha5)*d6],
                         [sin(q6)*sin(alpha5),cos(q6)*sin(alpha5),cos(alpha5),cos(alpha5)*d6],
                         [0,0,0,1]])

            T5_6=T5_6.subs(S)
            T6_7= Matrix([[cos(q7),-sin(q7),0,a6],
                         [sin(q7)*cos(alpha6),cos(q7)*cos(alpha6),-sin(alpha6),-sin(alpha6)*d7],
                         [sin(q7)*sin(alpha6),cos(q7)*sin(alpha6),cos(alpha6),cos(alpha6)*d7],
                         [0,0,0,1]])
            T6_7=T6_7.subs(S)
          #Correcting gripper cordinate by 180 on Z-axis and 90 on y-axis
            Rz=Matrix([[cos(np.pi),-sin(np.pi),0,0],
                      [sin(np.pi),cos(np.pi),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

            Ry=Matrix([[cos(-np.pi/2),0,sin(-np.pi/2),0],
                      [0,1,0,0],
                      [-sin(-np.pi/2),0,cos(-np.pi/2),0],
                      [0,0,0,1]])
            R_correction=(Rz*Ry)
           #Getting Transformation from base orgin 
            T0_1=(T0_1) 
            T0_2=(T0_1*T1_2)
            T0_3=(T0_2*T2_3)
            T0_4=(T0_3*T3_4)
            T0_5=(T0_4*T4_5)
            T0_6=(T0_5*T5_6)
            T0_7=(T0_6*T6_7)
            T0_gripper=(T0_7*R_correction)
           #Checking if values are correct for forward kinematics 
            #print 'T0_1'
            #print (T0_1.evalf(subs={q1:0}))
            #print 'T0_2'
            #print (T0_2.evalf(subs={q1:0,q2:0}))
            #print 'T0_3'
            #print (T0_3.evalf(subs={q1:0,q2:0,q3:0}))
            #print 'T0_4'
            #print(T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0}))
            #print 'T0_5'
            #print(T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0,q5:0}))
            #print 'T0_6'
            #print(T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0,q5:0,q6:0}))
            #print 'T0_7'
            #print(T0_7.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0,q5:0,q6:0,q7:0}))
            #print 'T0_gripper'
            #print(T0_gripper.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0,q5:0,q6:0,q7:0}))
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            P=Matrix([[px],
                     [py],
                     [pz],
                     [0]])
            D=Matrix([[d7],
                     [0],
                     [0],
                     [0]])
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
        
            print '-------------------------------------RESULTS-------------------------------------------------'
            print  '--------------------------------------------------------------------------------------------'
            print 'End Effector Orientation: Roll - Pitch - Yaw : '
            print 'Roll  : ',roll
            print 'Pitch : ',pitch
            print 'Yaw   : ',yaw
            print 'End Effector Position  : Px - Py - Pz : '
            print 'PX    : ',px
            print 'PY    : ',py
            print 'PZ    : ',pz

           # Calculate joint angles using Geometric IK method
            Ryaw=Matrix([[cos(yaw),-sin(yaw),0,0],
                      [sin(yaw),cos(yaw),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

            Rpitch=Matrix([[cos(pitch),0,sin(pitch),0],
                      [0,1,0,0],
                      [-sin(pitch),0,cos(pitch),0],
                      [0,0,0,1]])

            Rroll=Matrix([[1,0,0,0],
                      [0,cos(roll),-sin(roll),0],
                      [0,sin(roll),cos(roll),0],
                      [0,0,0,1]])

            R0_6=Ryaw*Rpitch*Rroll
            print  '--------------------------------------------------------------------------------------------'
            print 'WRIST  CENTER  POSITION : '
            W=P-R0_6*D
            W=W.subs(S)
            W_x=W[0]
            W_y=W[1]
            W_z=W[2]
            print 'Wx : ',W_x
            print 'Wy : ',W_y
            print 'Wz : ',W_z
            print  '--------------------------------------------------------------------------------------------'
            print 'VALUES  THETA 1 - 2 - 3 - 4 - 5 -6'
            #Calculating theta1
            theta1=atan2(W_y,W_x)
            print'THETA 1 : ',theta1
            #Calculating theta2 
            r=sqrt(W_x*W_x+W_y*W_y)-a1
            r=r.subs(S)
            s=W_z-d1
            s=s.subs(S)
            g=sqrt(r*r+s*s)
            alpha=atan2(s,r)
            #Calculating distance between joints 3 and 4
            d3_4=sqrt(d4*d4+a3*a3)
            d3_4=d3_4.subs(S)
            l=((a2*a2+g*g-d3_4*d3_4)/(2*g*a2))
            l=l.subs(S)
            beta=acos(l)
            beta.subs(S)
            theta2=(np.pi/2-(abs(alpha)+beta))
            print 'THETA 2 :',theta2
            #Calculating theta 3
            k=((a2*a2+d3_4*d3_4-g*g)/(2*d3_4*a2))
            k.subs(S)
            sigma=acos(k)
            sigma=sigma.subs(S)
            gamma=atan2(d4,a3)
            gamma=gamma.subs(S)
            theta3=(+np.pi-(sigma+gamma))
            theta3=theta3.subs(S)
            print'THETA 3 :',theta3
            #Obtaining R3_6
            T0_3=T0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
            R0_3=T0_3[:3,:3]
            R3_6=(R0_3.inv())*R0_6[:3,:3]
            R3_6_numpy=np.array(R3_6).astype(np.float64)
            alpha,beta,gamma=tf.transformations.euler_from_matrix(R3_6_numpy,axes='ryzx')
            theta4=alpha
            theta5=(beta-np.pi/2)
            theta6=gamma-np.pi/2
            print'THETA 4 :',theta4
            print'THETA 5 :',theta5
            print'THETA 6 :',theta6
            if theta5==0:
               theta4=0
               theta6=0
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
