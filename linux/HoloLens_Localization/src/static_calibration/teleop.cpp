//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "teleop.h"
#include <naoqi_bridge_msgs/BodyPoseActionGoal.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

//
// see https://github.com/ros-naoqi/nao_extras/blob/master/nao_teleop/src/teleop_nao_joy.cpp

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
teleop::teleop() :
    m_fStiffness(false),
    m_maxHeadYaw(2.0943), m_maxHeadPitch(0.7853),
    m_bodyPoseTimeOut(5.0),
    m_bodyPoseClient("body_pose", true)
{
    ros::NodeHandle nh;

    m_movePub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    m_headPub = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles", 1);
 
    m_stiffnessDisableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/disable");
    m_stiffnessEnableClient = nh.serviceClient<std_srvs::Empty>("body_stiffness/enable");
    m_wakeup = nh.serviceClient<std_srvs::Empty>("wakeup");
    m_rest = nh.serviceClient<std_srvs::Empty>("rest");
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
bool teleop::callBodyPoseClient(const std::string& poseName)
{
    if (!m_bodyPoseClient.isServerConnected()) {
        return false;
    }

    naoqi_bridge_msgs::BodyPoseGoal goal;

    goal.pose_name = poseName;

    m_bodyPoseClient.sendGoalAndWait(goal, m_bodyPoseTimeOut);

    actionlib::SimpleClientGoalState state = m_bodyPoseClient.getState();
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("Pose action \"%s\" did not succeed (%s): %s", goal.pose_name.c_str(), state.toString().c_str(), state.text_.c_str());
        return false;
    } else {
        ROS_INFO("Pose action \"%s\" succeeded", goal.pose_name.c_str());
        return true;
    }
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void teleop::operation(int c)
{
  	if (c == 'i') { // move forward
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = 0.1;
        msgTwist.linear.y = 0;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = 0;
        m_movePub.publish(msgTwist);
	} else if (c == 'j') { // move left
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = 0;
        msgTwist.linear.y = 0.1;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = 0;
        m_movePub.publish(msgTwist);
	} else if (c == 'l') { // move right
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = 0;
        msgTwist.linear.y = -0.1;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = 0;
        m_movePub.publish(msgTwist);
	} else if (c == 'u') { // turn left
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = 0;
        msgTwist.linear.y = 0;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = 0.2;
        m_movePub.publish(msgTwist);
	} else if (c == 'o') { // turn right
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = 0;
        msgTwist.linear.y = 0;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = -0.2;
        m_movePub.publish(msgTwist);
	} else if (c == 'k') { // move backward
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = -0.1;
        msgTwist.linear.y = 0;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = 0;
        m_movePub.publish(msgTwist);
	} else if (c == 's') { // stop
        geometry_msgs::Twist msgTwist;
        msgTwist.linear.x = 0;
        msgTwist.linear.y = 0;
        msgTwist.linear.z = 0;
        msgTwist.angular.x = 0;
        msgTwist.angular.y = 0;
        msgTwist.angular.z = 0;
        m_movePub.publish(msgTwist);
	}
    else if (c == 'p')
    {
        std_srvs::Empty e;

        m_fStiffness = !m_fStiffness;

        if (m_fStiffness) {
            std::cout << "enabling stiffness\r\n" << std::endl;
            m_stiffnessEnableClient.call(e);
        } else { 
            std::cout << "disabling stiffness\r\n" << std::endl;
            m_stiffnessDisableClient.call(e);
        }
    }
    else if (c == 'e')
    {
        std_srvs::Empty e;
        m_wakeup.call(e);
    }
    else if (c == 'r')
    {
        std_srvs::Empty e;
        m_rest.call(e);
    }
    else if (c == '0')
    {
        setPepperHeadPitchYaw(0.0, 0.0);
    }
    else if (c == '1')
    {
        setPepperHeadPitchYaw(-0.35, 0.0);
    }
    else if (c == '2')
    {
        setPepperHeadPitchYaw(0.0, 0.7);
    }
    else if (c == '3')
    {
        setPepperHeadPitchYaw(0.0, -0.7);
    }
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void teleop::setPepperHeadPitch(float angle)
{
    naoqi_bridge_msgs::JointAnglesWithSpeed     headpose;

    headpose.joint_names.clear();
    headpose.joint_names.push_back("HeadPitch");
    headpose.joint_angles.clear();
    headpose.joint_angles.push_back(angle);
    headpose.speed = 0.1;
    headpose.relative = 0; // ABSOLUTE MODE

    m_headPub.publish(headpose);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void teleop::setPepperHeadYaw(float angle)
{
    naoqi_bridge_msgs::JointAnglesWithSpeed     headpose;

    headpose.joint_names.clear();
    headpose.joint_names.push_back("HeadYaw");
    headpose.joint_angles.clear();
    headpose.joint_angles.push_back(angle);
    headpose.speed = 0.1;
    headpose.relative = 0; // ABSOLUTE MODE

    m_headPub.publish(headpose);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void teleop::setPepperHeadPitchYaw(float pitch, float yaw)
{
    naoqi_bridge_msgs::JointAnglesWithSpeed     headpose;

    headpose.joint_names.clear();
    headpose.joint_names.push_back("HeadPitch");
    headpose.joint_names.push_back("HeadYaw");
    headpose.joint_angles.clear();
    headpose.joint_angles.push_back(pitch);
    headpose.joint_angles.push_back(yaw);
    headpose.speed = 0.1;
    headpose.relative = 0; // ABSOLUTE MODE

    m_headPub.publish(headpose);
}
