#!/usr/bin/env python
import math
import copy
from dataclasses import asdict
import moveit_commander, geometry_msgs.msg, moveit_msgs.msg, rospy, sys
from std_msgs.msg import Float64MultiArray
import numpy as np
import roslib
from time import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trac_ik_python.trac_ik import IK

roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

class RobotControl:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)

        # definirati instancu
        self.robot = moveit_commander.RobotCommander()

        # definirati scenu
        self.scene = moveit_commander.PlanningSceneInterface()

        # stvoriti objekt koji pruza sucelje za planiranje pomaka za definiranu grupu u setup assistant-u
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.get_current_pose()

        # gripper publisher
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)

        self.ik = IK('base_link', 'tool0', solve_type="Distance")


    def setPosition(self, x, y, z, angle):

        x = float(x)
        y = float(y)
        z = float(z)

        # gripper face down
        qx = 1
        qy = 0
        qz = 0
        qw = 0

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)

        print("Inverse kin:")
        print(goal_joints)

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joints[0]
        joint_goal[1] = goal_joints[1]
        joint_goal[2] = goal_joints[2]
        joint_goal[3] = goal_joints[3]
        joint_goal[4] = goal_joints[4]
        joint_goal[5] = goal_joints[5]+angle

            
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
    
    def setPositionAndOrientation(self, position, orientation):

        x = float(position.x)
        y = float(position.y)
        z = float(position.z)

        # gripper orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)

        print("Inverse kin:")
        print(goal_joints)

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joints[0]
        joint_goal[1] = goal_joints[1]
        joint_goal[2] = goal_joints[2]
        joint_goal[3] = goal_joints[3]
        joint_goal[4] = goal_joints[4]
        joint_goal[5] = goal_joints[5]

            
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
    


    def setPositionUpDown(self, delta_z):

        t_dots=RC.move_group.get_current_pose().pose.position

        x = float(t_dots.x)
        y = float(t_dots.y)
        z = float(t_dots.z+delta_z)

        # gripper face down + object orientation
        Q=RC.move_group.get_current_pose().pose.orientation
        qx = Q.x
        qy = Q.y
        qz = Q.z
        qw = Q.w

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)

        print("Inverse kin:")
        print(goal_joints)

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joints[0]
        joint_goal[1] = goal_joints[1]
        joint_goal[2] = goal_joints[2]
        joint_goal[3] = goal_joints[3]
        joint_goal[4] = goal_joints[4]
        joint_goal[5] = goal_joints[5]

            
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()
    

    def openGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 0
        
        
        # Delay od 3 sekunde da se saka otvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break

    def closeGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 100
        
        # Delay od 3 sekunde da se saka zatvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break
    
    def executeTrajectory(self, plan):
        self.move_group.execute(plan, wait=True)
    

def currentPose_quaternionToRotation():  
    Q=RC.move_group.get_current_pose().pose.orientation
    t_dots=RC.move_group.get_current_pose().pose.position
    print(Q)
    print(t_dots)
    t=np.array([[t_dots.x],
                [t_dots.y],
                [t_dots.z]])
    print(t)
        # Extract the values from Q
    q0 = Q.w
    q1 = Q.x
    q2 = Q.y
    q3 = Q.z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    R = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    TAB=np.concatenate([R, t], axis=1)
    TAB=np.concatenate([TAB, [np.array([0,0,0,1])]])
    return TAB


if __name__ == '__main__':
    
    # robot control instance

    RC = RobotControl()

    print("Waiting for messages on topics /matrix_R and /matrix_t")
    Matrix_R=Float64MultiArray()
    Matrix_t=Float64MultiArray()
    Matrix_R=rospy.wait_for_message("/matrix_R", Float64MultiArray)
    print("Got matrix R")
    Matrix_t=rospy.wait_for_message("/matrix_t", Float64MultiArray)
    print("Got matrix t")
    objectSize=rospy.wait_for_message("/size", Float64MultiArray)
    print("Got object size")

    objectSize=np.reshape(objectSize.data, (3,1))

    #Reshaping matrices R and t
    Matrix_R=np.reshape(Matrix_R.data, (3,3))
    Matrix_t=np.reshape(Matrix_t.data, (3,1))
    TOC=np.concatenate([Matrix_R, Matrix_t], axis=1)
    TOC=np.concatenate([TOC, [np.array([0,0,0,1])]])
    print("Matrix TOC")
    print(TOC)

    TCA=np.array([[1, 0, 0, 0],[0, 1, 0, -0.195],[0, 0, 1, -0.035],[0, 0, 0, 1]])

    print("Matrix TCA")
    print(TCA)

    TAB=currentPose_quaternionToRotation()
    print("Matrix TAB")
    print(TAB)

    TOB =TAB@TCA@TOC

    print("Matrix TOB")
    print(TOB)

    tOB = TOB[0:3, 3]

    print("Point tOB")
    print(tOB)

    #save current position for later use
    original_orientation=RC.move_group.get_current_pose().pose.orientation
    original_position=RC.move_group.get_current_pose().pose.position

    input("Press enter to continue")
    # offset above the object in m
    offset=0.45

    RC.setPosition(tOB[0], tOB[1], tOB[2]+offset, 0)
    print("Finished ", offset ,"m above object")
    input("Succesful? Press enter to continue")

    xOC=TOC[0:4,0]
    print("Matrix xOC:")
    print(xOC)

    # get xAB and xOB dots
    xOB=TAB@TCA@xOC
    TAB=currentPose_quaternionToRotation()
    xAB=TAB[0,0:3]

    #calculate angle between axes
    theta_1=-math.acos(xAB[0])
    theta_2=math.acos(xOB[0])
    theta=theta_2-theta_1-math.pi/4

    print("Angle theta_1(xAB):")
    print(theta_1)
    print("Angle theta_2(xOB):")
    print(theta_2)

    print("Angle theta(theta_2-theta_1):")
    print(theta)

    input("Are angles okay? Press enter to continue")

    #Align gripper
    RC.setPosition(tOB[0], tOB[1], tOB[2]+offset, theta)

    print("Finished aligning gripper")
    input("Is the gripper aligned? Press enter to continue")

    # Open gripper
    RC.openGripper()

    # Calculate height to come down and start grabbing object
    delta_z=offset-objectSize[2]/2-0.15
    RC.setPositionUpDown(-delta_z)
    print("Finished coming down to object")
    input("Is robot ready to grip the object? Press enter to continue")

    # Close gripper
    RC.closeGripper()
    print("Caught the object")

    RC.setPositionUpDown(delta_z+0.1)
    print("Lifted the object")

    input("Can the robot drop the object? Press enter to continue")
    RC.openGripper()
    print("Dropped the object")

    input("Can the robot go back to original position? Press enter to continue")
    #Go back to starting position
    RC.setPositionAndOrientation(original_position,original_orientation)

    print("Sequence done!")

    rospy.spin()
 
