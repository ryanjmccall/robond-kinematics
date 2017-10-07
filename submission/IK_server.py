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


def tfMatrix(alpha, a, d, q):
    """Returns a DH transformation matrix based on DH params"""
    return Matrix([[cos(q), -sin(q), 0, a],
                   [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha),
                    -sin(alpha) * d],
                   [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha),
                    cos(alpha) * d],
                   [0, 0, 0, 1]])


class ForwardKinematics():
    """Contains pre-calculated forward kinematics transformations and variables.
    """
    def __init__(self):
        # Create symbols
        # DH param symbols
        # twist angles
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols(
            "alpha0:7")

        # link lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")

        # link offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")

        # joint angles
        self.q1, self.q2, self.q3, q4, q5, q6, q7 = symbols("q1:8")

        # Create Modified DH parameters
        dhTable = {alpha0: 0, a0: 0, d1: 0.75, self.q1: self.q1,
                   alpha1: -pi / 2., a1: 0.35, d2: 0,
                   self.q2: self.q2 - pi / 2.,
                   alpha2: 0, a2: 1.25, d3: 0, self.q3: self.q3,
                   alpha3: -pi / 2., a3: -0.054, d4: 1.5, q4: q4,
                   alpha4: pi / 2., a4: 0, d5: 0, q5: q5,
                   alpha5: -pi / 2., a5: 0, d6: 0, q6: q6,
                   alpha6: 0, a6: 0, d7: .303, q7: 0}

        # Define Modified DH Transformation matrix
        # Create individual transformation matrices
        self.t01 = tfMatrix(alpha0, a0, d1, self.q1).subs(dhTable)
        self.t12 = tfMatrix(alpha1, a1, d2, self.q2).subs(dhTable)
        self.t23 = tfMatrix(alpha2, a2, d3, self.q3).subs(dhTable)
        t34 = tfMatrix(alpha3, a3, d4, q4).subs(dhTable)
        t45 = tfMatrix(alpha4, a4, d5, q5).subs(dhTable)
        t56 = tfMatrix(alpha5, a5, d6, q6).subs(dhTable)
        t6EE = tfMatrix(alpha6, a6, d7, q7).subs(dhTable)
        t0EE = self.t01 * self.t12 * self.t23 * t34 * t45 * t56 * t6EE

        #
        # Extract rotation matrices from the transformation matrices
        #
        # end-effector rot matrix
        r, p, y = symbols("r p y")

        # x <=> rol
        rotX = Matrix([[1, 0, 0],
                       [0, cos(r), -sin(r)],
                       [0, sin(r), cos(r)]])
        # y <=> pitch
        rotY = Matrix([[cos(p), 0, sin(p)],
                       [0, 1, 0],
                       [-sin(p), 0, cos(p)]])
        # z <=> yaw
        rotZ = Matrix([[cos(y), -sin(y), 0],
                       [sin(y), cos(y), 0],
                       [0, 0, 1]])

        rotEE = rotZ * rotY * rotX
        # Error correction; align DH parameters with those in URDF file
        rotError = rotZ.subs(y, radians(180)) * rotY.subs(p, radians(-90))

        # end-effector rotation matrix
        self.correctedRotEE = rotEE * rotError


print("init FK...")
_fwdKin = ForwardKinematics()
print("init FK complete")

###
# SSS triangle constants
SIDE_A = 1.501
SIDE_C = 1.25


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

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # roll, pitch, yaw = end-effector orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and
            # Gazebo
            #
            # Calculate joint angles using Geometric IK method
            ###
            ################################
            ROT_EE = _fwdKin.correctedRotEE.subs({"r": roll, "p": pitch,
                                                  "y": yaw})

            # end-effector position

            # calculate wrist center position from end-effector position!
            # formula: 0_r_WC/0 = 0_r_EE/0 - d * 0-6R * [[0],[0][1]] -
            # [[px], [py], [pz]] - d * 0-6R [[0][0][1]]
            WC = Matrix([[px], [py], [pz]]) - 0.303 * ROT_EE[:, 2]

            # Calculate joint angles using geometric IK method
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3
            side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2) +
                          pow(WC[2] - 0.75, 2))

            angle_a = acos((side_b * side_b + SIDE_C * SIDE_C -
                            SIDE_A * SIDE_A) / (2 * side_b * SIDE_C))
            angle_b = acos((SIDE_A * SIDE_A + SIDE_C * SIDE_C -
                            side_b * side_b) / (2 * SIDE_A * SIDE_C))
            # angle_c = acos((SIDE_A * SIDE_A + side_b * side_b -
            # SIDE_C * SIDE_C) / (2 * SIDE_A * side_b))

            theta2 = pi / 2. - angle_a - atan2(WC[2] - 0.75,
                                               sqrt(WC[0] * WC[0] +
                                                    WC[1] * WC[1]) - 0.35)

            # constant account for sag in link4 of -0.054m
            theta3 = pi / 2. - (angle_b + 0.036)

            r03 = (_fwdKin.t01[0:3, 0:3] * _fwdKin.t12[0:3, 0:3] *
                   _fwdKin.t23[0:3, 0:3])
            r03 = r03.evalf(subs={_fwdKin.q1: theta1, _fwdKin.q2: theta2,
                                  _fwdKin.q3: theta3})
            r36 = r03.inv("LU") * ROT_EE

            # compute Euler angles from the rotation matrix from 3-6
            theta4 = atan2(r36[2, 2], -r36[0, 2])
            theta5 = atan2(sqrt(r36[0, 2] * r36[0, 2] + r36[2, 2] * r36[2, 2]),
                           r36[1, 2])
            theta6 = atan2(-r36[1, 1], r36[1, 0])

            ################################
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint
            # angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4,
                                                theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" %
                  len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()