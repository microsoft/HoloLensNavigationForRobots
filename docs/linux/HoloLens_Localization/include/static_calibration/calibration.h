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
#include <boost/concept_check.hpp>
#include <boost/thread.hpp>
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigen>
#include <iostream>
#include<fstream>
#include "teleop.h"
#include <sys/ioctl.h>
#include <termios.h>
#include "cost_function.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace Eigen;

class calibrator
{
public:
    calibrator(std::string holoLinkedFrame, std::string odomFrame, std::string robotFootFrame,std::string fpath);
    void run();
private:
    void publish_thread();

    void setTransform();
    void calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T);
    void horizontalCalibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T);
    void transition_log(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos,std::ofstream& ofs);	
    void linear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T);
    void nonlinear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d& R, Vector3d& T);	
    void horizontal_initialization(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T, double* bestScores);
    void nonlinear_horizontal_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d& R, Vector3d& T, double* bestScores);
    void poseStampedCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool lookupTransform();
    void recordCurrentPosition();
    void calibrate();
    bool setHeadPositionAndRecord(float fPitch, float fYaw);
    bool autocalibrate();
    void writeCalibrationData();

    std::vector<tf::StampedTransform>                       m_pep_pos;
    std::vector<tf::StampedTransform>                       m_hol_pos;
    std::vector<Eigen::Vector3d>                            m_floor2holo;
    std::vector<Eigen::Vector3d>                            m_head2foot;

    geometry_msgs::PoseStamped                              m_latestPoseStamped;

    Matrix3d R;
    Vector3d T;

    tf::StampedTransform                                    m_transform1;
    tf::StampedTransform                                    m_transform2;
    tf::StampedTransform                                    m_transform3;
    tf::StampedTransform                                    m_transform4;
    tf::TransformBroadcaster                                m_tf_br;
    tf::StampedTransform                                    m_tf_map_to_odom;
    ros::Subscriber                                         m_holo_floor_sub;
    ros::NodeHandle                                         m_nh;

    bool                                                    m_horizontalCalibMode = false;

    std::string                                             m_holoLinkedFrame;
    std::string                                             m_odomFrame;
    std::string                                             m_robotFootFrame;
    std::string                                             m_fpath;

    tf::TransformListener                                   m_listener;
    teleop                                                  m_tele;

	int getch()
	{
		static struct termios oldt, newt;
		tcgetattr( STDIN_FILENO, &oldt);           // save old settings
		newt = oldt;
		newt.c_lflag &= ~(ICANON);                 // disable buffering
		tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

		int c = getchar();  // read character

		tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
		return c;
	}

    bool kbhit()
	{
		termios term;
		tcgetattr(0, &term);

		termios term2 = term;
		term2.c_lflag &= ~ICANON;
		tcsetattr(0, TCSANOW, &term2);

		int byteswaiting;
		ioctl(0, FIONREAD, &byteswaiting);

		tcsetattr(0, TCSANOW, &term);

		return byteswaiting > 0;
	}
	
	Matrix4d btTrans2EigMat4d(tf::Transform t)
    {
		tf::Matrix3x3 btm(t.getRotation());
		Matrix4d m;
		m<<btm.getRow(0).getX(),btm.getRow(0).getY(),btm.getRow(0).getZ(),t.getOrigin().getX(),
		btm.getRow(1).getX(),btm.getRow(1).getY(),btm.getRow(1).getZ(),t.getOrigin().getY(),
		btm.getRow(2).getX(),btm.getRow(2).getY(),btm.getRow(2).getZ(),t.getOrigin().getZ(),
		0,0,0,1;
		return m;
	}

	Vector3d mat2axis(Matrix4d m)
    {
		double x,y,z;
		double r=sqrt((m(2,1)-m(1,2))*(m(2,1)-m(1,2))+(m(0,2)-m(2,0))*(m(0,2)-m(2,0))+(m(1,0)-m(0,1))*(m(1,0)-m(0,1)));
		x=(m(2,1)-m(1,2))/r;
		y=(m(0,2)-m(2,0))/r;
		z=(m(1,0)-m(0,1))/r;
		Vector3d t;
		t<<x,y,z;
		return t;
	}

	void mat2axis_angle(Matrix4d m,Vector3d& retv, double& angle)
    {
		double x,y,z;
		double r=sqrt((m(2,1)-m(1,2))*(m(2,1)-m(1,2))+(m(0,2)-m(2,0))*(m(0,2)-m(2,0))+(m(1,0)-m(0,1))*(m(1,0)-m(0,1)));
		x=(m(2,1)-m(1,2))/r;
		y=(m(0,2)-m(2,0))/r;
		z=(m(1,0)-m(0,1))/r;
		Vector3d t;
		t<<x,y,z;
		retv=t;
		angle=acos((m(0,0)+m(1,1)+m(2,2)-1)/2);
	}
};
