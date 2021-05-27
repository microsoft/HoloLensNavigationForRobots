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

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/concept_check.hpp>
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include<fstream>

#include "calibration.h"

using namespace Eigen;

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void printHelp()
{
    std::cout<<"---------------------------------------------------------"<<std::endl
      <<"Robot-HoloLens calibration"<<std::endl
      <<"  space: record position"<<std::endl
      <<"  d: delete last recorded position"<<std::endl
      <<"  c: conduct calibration"<<std::endl
      <<"  a: auto calibration (Pepper Robot)"<<std::endl
      <<"  w: write calibration data"<<std::endl
      <<"  z: write calibration parameter log as text file"<<std::endl
      <<"  t: toggle calibration mode (clear all recorded position)"<<std::endl
      <<"  h,?: print this help"<<std::endl
      <<"  q: quit calibration tool"<<std::endl
      << std::endl
      <<"Robot commands"<<std::endl
      <<"  i,j,k,l: move robot forward, left, backward, right"<<std::endl
      <<"  u,o: rotate robot counter-clockwise, clockwise"<<std::endl
      <<"  s: stop robot movement"<<std::endl
      << std::endl
      <<"Pepper commands"<<std::endl
      <<"  e: wakeup"<<std::endl
      <<"  r: rest"<<std::endl
      <<"  p: toggle stiffness"<<std::endl
      <<"  0,1,2,3: various head positions"<<std::endl
      <<"---------------------------------------------------------"<<std::endl;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "static_calibration");

    if (argc < 3) {
        std::cout<<"Usage: static_calibration <robot's odom frame name> <robot's frame name linking to hololens> <robot's foot frame name> <optional: calibration file>"<<std::endl;
        return 0;
    }

    std::string odomFrame(argv[1]);
    std::string holoLinkedFrame(argv[2]);	
    std::string robotFootFrame(argv[3]);
    std::string fpath;

    if (argc >= 5) {
        fpath = std::string(argv[4]);
    }

    printHelp();

    // set up publish rate
    calibrator calib(holoLinkedFrame, odomFrame, robotFootFrame, fpath);

    calib.run();

    return 0;
}
