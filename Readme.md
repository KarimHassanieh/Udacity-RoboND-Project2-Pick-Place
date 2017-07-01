## Project: Kinematics Pick & Place
### Readme : This file is meant to guide the reviewer along the steps and methods taken to accomplish the project in hope of clarifying to the reviewer the steps taken with clarity.
---


**Steps to done and needed to complete the project:**  


1. Setting up  ROS Workspace.
2. Downloading or cloning the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launching  [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Performing Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./Results/kuka_arm_frame.png
[image2]: ./Results/demo.png
[image3]: ./Results/matrix.png
[image4]: ./Results/q1.png
[image5]: ./Results/full_solutions.png
[image6]: ./Results/front_above_position.png
[image7]: ./Results/can_in_bin.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This write up has been provided.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The forward kinematics demo was run and used to evauluate the joint parameter. Below image shows demo succefully running in ROS
![alt text][image2]

Moreover this was the basis of evaluating the kinematic frames and there distribution. The below image shows how the frames were distributed

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

link | alpha j-1 | a i-1 | d i-1 | Theta i
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | Theta 1
2 | -pi | 0.35 | 0| Theta 2
3 | 0 | 1.25 | 0 | Theta 3
4 | -pi | -0.054 | 1.5 | Theta 4 
5 | pi | 3 | 0 | Theta 5
6 | -pi| 3 | 0 | Theta 6
7 | 0 | 3 | 0.303 | 0

The homogenous transformation was then calculated by inserting the below formula (refernced from Part 13 of the Udacity -Pick and Place Project section) to have transfomation from i-1 to i , in order to obtain the overall transform between the base_link and gripper_link the consecutive transformation matrices were multiplied (T base to gripper = Tbase to 1 * T 1 to 2 * T 2 to 3 * T3 to 4 * T5 to 6 * T 6 to gripper). Also to note after obtaining T base to gripper a correction Rotation matrix was implemented around z and y axis to align the axis.The final transformation matrix are part of the code presented in the IK_server.py code.


![alt text][image3]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

In order to approach the Inverse Kinematics problem initailly we will start by computing the wrist center. 
The wrist center will be calculated using roll , pitch , yaw values obtained. Using these angle R 0_6 is evaluated.
(R 0_6 = Ryaw around zaxis* Rpitch around y axis * R roll around x axis )
 
 Wrist center position is then calculated based on the following formula
 (Wrist position = end effector position obtained from ROS - R 0_6 * end effector length where end effector length is equal to 0.303) 
In order to calculate thetas : 
- For Theta 1 :
Wrist position is projected on x-y axis similair to the image below . Theta 1 is then calculated using atan2(Wc projection on y axis, Wc projection on x axis ) 

![alt text][image4]

Moving on we have to realize there are several possible configurations and acceptable mathematical & trigometric solutions  as presented below based on the values chosen. For our purpose position -A (Front and above position ) is the most acceptable solution for our given workspace.

![alt text][image5]

- For Theta 3 :
Wrist position is projected on x-y axis similair to the image below . Theta 1 is then calculated using atan2(Wc projection on y axis, Wc projection on x axis ) 

![alt text][image6]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image7]


