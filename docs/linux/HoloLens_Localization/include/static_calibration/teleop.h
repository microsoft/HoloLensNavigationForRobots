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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/BodyPoseAction.h>
#include <actionlib/client/simple_action_client.h>

class teleop
{
public:
    teleop();

    void operation(int c);
    void setPepperHeadPitch(float angle);
    void setPepperHeadYaw(float angle);
    void setPepperHeadPitchYaw(float pitch, float yaw);

private:
    /**
    * \brief calls m_bodyPoseClient on the poseName, to execute a body pose
    * @return success of actionlib call
    */
    bool callBodyPoseClient(const std::string& poseName);

private:
    bool                                                    m_fStiffness;
    double                                                  m_maxHeadYaw;
    double                                                  m_maxHeadPitch;
    ros::Duration                                           m_bodyPoseTimeOut;
    ros::Publisher                                          m_movePub;
    ros::Publisher                                          m_headPub;
    ros::ServiceClient                                      m_stiffnessDisableClient;
    ros::ServiceClient                                      m_stiffnessEnableClient;
    ros::ServiceClient                                      m_wakeup;
    ros::ServiceClient                                      m_rest;
    actionlib::SimpleActionClient<naoqi_bridge_msgs::BodyPoseAction> m_bodyPoseClient;
};

