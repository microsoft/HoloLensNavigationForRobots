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
#include "ICP_module.h"

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
int main(int argc, char* argv[])
{
    ros::init(argc,argv, "anchor_localizer");
    ros::NodeHandle n;

    ICP2D_Module icp_module(n);

    icp_module.start();
    
    return 0;
}
