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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <map>

typedef std::map<std::string,tf::StampedTransform> MTF;

tf::StampedTransform positionMat2tf(float* posdata);
tf::StampedTransform EigenMat2tf(Eigen::Matrix4d posdataMat);
Eigen::Matrix4d tf2EigenMat(tf::StampedTransform tfdata);
