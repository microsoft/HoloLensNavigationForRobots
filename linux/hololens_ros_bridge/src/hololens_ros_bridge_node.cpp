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
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud.h"
#include "data_connection.h"
#include "funcs.h"

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

const char * pszAppName = "hololens_ros_bridge";
bool sendMsg = false;
bool recvTF = false;
float keptHeight;
tf::StampedTransform alignedTF;
tf::StampedTransform tf_hololens2ros;

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void callbackTest(const std_msgs::Bool::ConstPtr &subscr_ping)
{
    ROS_INFO("Request to initialize HoloLens bridge received.");
    sendMsg = true;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void callback_aligned(const geometry_msgs::PoseStamped::ConstPtr &ps)
{
    //
    // callback for receiving alignment result

    ROS_INFO("Request to align localized spatial anchor received.");

    // convert PoseStamped to tf
    alignedTF.setOrigin(tf::Vector3(ps->pose.position.x, ps->pose.position.y, keptHeight));

    tf::Quaternion initq(
        ps->pose.orientation.x,
        ps->pose.orientation.y,
        ps->pose.orientation.z,
        ps->pose.orientation.w);

    alignedTF.setRotation(initq);

    tf::Transform tf_map2sa = alignedTF;

    alignedTF.setData(tf_map2sa);
	alignedTF.stamp_=ros::Time::now();
    alignedTF.frame_id_ = std::string("map");
    alignedTF.child_frame_id_ = std::string("spatialAnchor_ros");

    recvTF = true;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
int main(int argc, char **argv)
{
    if (argc < 3)
    {
        ROS_ERROR("missing parameters: <ip address> <port");
        return -1;
    }

    // initialization
    ros::init(argc, argv, pszAppName);

    ros::NodeHandle nh;

    // initial localization request
    ros::Subscriber testsubsc = nh.subscribe("/hololens/ack", 1000, callbackTest);

    // alignement result from anchor localizer
    ros::Subscriber locsubsc = nh.subscribe("/hololens/localized", 1000, callback_aligned);

    // point cloud from HoloLens
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("/hololens/pc", 1000);

    // floor surface normal
    ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseStamped>("/hololens/floor_normal", 1000);

    // port, ip_address of HoloLens (port should be 1234)
    const int                   port = std::stoi(argv[2]);
    const char *                ip_address = argv[1];

    ROS_INFO("Socket trying to connect to %s:%i...", ip_address, port);

    int sock = socket(PF_INET, SOCK_STREAM, 0);

    // connection
    struct sockaddr_in addr;
    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip_address);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        ROS_FATAL("Socket connection error.");
        return -1;
    }

    ROS_INFO("Socket successfully connected.");

    EnumAskState                askState;
    EnumRepliedState            repState;
    char                        recvBuf[2048];
    float                       cameraPosition[20];
    float                       holoLensHeight;
    Eigen::Vector3d             toFloor=Eigen::Vector3d::Zero();

    // Spatial Anchor (SA) in ROS - SA in HoloLens (Rotation alignment)
    Eigen::Matrix4d             hololens2ros;

    hololens2ros << 0, 0, -1, 0,
        -1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 0, 1;

    // transformation chain: map - SA (ROS) - SA (HoloLens) - HoloLens camera
    tf::StampedTransform        tf_anchor2camera;
    tf::StampedTransform        tf_map2anchor;

    tf_hololens2ros = EigenMat2tf(hololens2ros);
    tf_hololens2ros.frame_id_ = std::string("spatialAnchor_ros");
    tf_hololens2ros.child_frame_id_ = std::string("spatialAnchor");

    tf_map2anchor = EigenMat2tf(Eigen::Matrix4d::Identity());
    tf_map2anchor.frame_id_ = std::string("map");
    tf_map2anchor.child_frame_id_ = std::string("spatialAnchor_ros");

    // tf broadcaster
    tf::TransformBroadcaster tf_br_;

    // string id - tf lookup table
    MTF                         transLookUp;

    // string id of current anchor
    std::string                 CurrentAnchor;
    bool                        transBool = false;
    sensor_msgs::PointCloud     pointcloud;

    while (ros::ok())
    {
        askState = Default_Ask_Pose;
        if (sendMsg)
        { // if receiving localization initialization from anchor localizer
            askState = Refresh_Anchor_And_Render_Map;
            transLookUp.clear();
            ROS_INFO("Refreshing point cloud data and anchors...");
            sendMsg = false;
        }

        if (recvTF)
        { // if receiving alignment result from anchor localizer
          // update tf of map-SA (ROS)
            transLookUp.at(CurrentAnchor) = alignedTF;
            tf_map2anchor = alignedTF;
            recvTF=false;
        }

        // send message to HoloLens
        int nRevAskState = reverseEndian(askState);
        int nWrite = write(sock, &nRevAskState, sizeof(EnumAskState));
        if (nWrite <= 0)
        {
            ROS_FATAL("Socket write failed (%d bytes).", nWrite);
            break;
        }

        // receive message from HoloLens
        int nRead = read(sock, (char *)&repState, sizeof(EnumRepliedState));
        if (nRead <= 0)
        {
            ROS_FATAL("Socket read failed (%d bytes).", nRead);
            break;
        }

        // received message
        repState = EnumRepliedState(reverseEndian(repState));

        switch (repState)
        {
            case Recv_Anchor_Pose_Map:
            { // receive point cloud
                // receive transformation of (old SA-new SA) in HoloLens
                recvMessage(sock, sizeof(float) * 16, (char *)cameraPosition);

                Eigen::Matrix4d anc2anc;
                anc2anc << cameraPosition[0], cameraPosition[4], cameraPosition[8], cameraPosition[12],
                    cameraPosition[1], cameraPosition[5], cameraPosition[9], cameraPosition[13],
                    cameraPosition[2], cameraPosition[6], cameraPosition[10], cameraPosition[14],
                    cameraPosition[3], cameraPosition[7], cameraPosition[11], cameraPosition[15];

                // obtain new SA position (temporal)
                Eigen::Matrix4d map2anc = tf2EigenMat(tf_map2anchor);
                std::cout << pszAppName << ": old anchor to new anchor " << std::endl;
                std::cout << /* std::setw(16) << */ anc2anc << std::endl;

                // tf:[map-new SA (ROS)] = (map - old SA) * (Ros-HoloLens) * (old SA-new SA) * (HoloLens - ROS)
                map2anc = map2anc * tf2EigenMat(tf_hololens2ros) * anc2anc * tf2EigenMat(tf_hololens2ros).inverse(); 
                std::cout << pszAppName << ": map to new anchor " << std::endl;
                std::cout << /* std::setw(16) << */ map2anc << std::endl;

                // update tf:[map-SA (ROS)]
                tf_map2anchor = EigenMat2tf(map2anc);
                tf_map2anchor.frame_id_ = std::string("map");
                tf_map2anchor.child_frame_id_ = std::string("spatialAnchor_ros");

                // receive point cloud
                char buffer[4], bufferswap[4];

                // size of point cloud buffer
                recvMessage(sock, 4, buffer);

                bufferswap[0] = buffer[3];
                bufferswap[1] = buffer[2];
                bufferswap[2] = buffer[1];
                bufferswap[3] = buffer[0];

                unsigned int        uTotalBufferSize = ((unsigned int *)bufferswap)[0];
                char *              vertexCollectionDataBuffer = (char *)malloc(sizeof(char) * uTotalBufferSize);

                recvMessage(sock, uTotalBufferSize, vertexCollectionDataBuffer);

                char *              ptr = vertexCollectionDataBuffer;
                unsigned int        currentBufferPosition = 0;

                pointcloud.points.clear();

                // read buffer
                while (true)
                {
                    // SurfaceMeshStreamHeader contains center, orientation, and vertex position length of point cloud 
                    SurfaceMeshStreamHeader *   hdr = (SurfaceMeshStreamHeader *)ptr;

                    if (memcmp(hdr->signature, (void *)SMSH_SIGNATURE, sizeof(hdr->signature)) != 0) {
                        ROS_ERROR("vertex collection buffer misalignement at offset %u.", currentBufferPosition);
                        break;
                    }
                    
                    Eigen::Vector3d             center;

                    center << hdr->center_x, hdr->center_y, hdr->center_z;

                    Eigen::Quaterniond          orientation = Eigen::Quaterniond(hdr->orientation_w, hdr->orientation_x, hdr->orientation_y, hdr->orientation_z);

                    unsigned int uNumVertices = hdr->uNumVertices;

                    // move to vertex buffer
                    ptr += sizeof(SurfaceMeshStreamHeader);

                    short *             vertexBuffer = (short *)ptr;
                    
                    // read point cloud
                    for (int i = 0; i < uNumVertices; i++)
                    {
                        Eigen::Vector3d p;

                        // point position
                        p << vertexBuffer[i * 4] * 0.0025f, vertexBuffer[i * 4 + 1] * 0.0025f, vertexBuffer[i * 4 + 2] * 0.0025f;

                        // point transformation
                        p = orientation * p + center;

                        // store in point cloud array
                        geometry_msgs::Point32 pt;

                        pt.x = p(0);
                        pt.y = p(1);
                        pt.z = p(2);

                        pointcloud.points.push_back(pt);
                    }

                    //move to next header
                    ptr += hdr->uVertexBufferSize;
                    currentBufferPosition += sizeof(SurfaceMeshStreamHeader) + hdr->uVertexBufferSize;
                    if (currentBufferPosition >= uTotalBufferSize)
                        break;
                }

                //
                // done with buffer, delete it
                // free(vertexCollectionDataBuffer);

                // ack for point cloud publish
                transBool = true;
                // continue to next process in (case Recv_Anchor_Pose:)
            }
            // fall through...

            case Recv_Anchor_Pose:
            { // receive anchor id
                char buffer[4], bufferswap[4];

                // anchor string id length
                recvMessage(sock, 4, buffer);
                bufferswap[0] = buffer[3];
                bufferswap[1] = buffer[2];
                bufferswap[2] = buffer[1];
                bufferswap[3] = buffer[0];

                unsigned int bufferLength = ((unsigned int *)bufferswap)[0];
                char *strBuf = (char *)malloc(sizeof(char) * bufferLength + 1);

                // Anchor string id
                recvMessage(sock, sizeof(char) * bufferLength, strBuf);
                strBuf[sizeof(char) * bufferLength] = 0;
                std::string str(strBuf);
                std::cout << pszAppName << ": anchor string is '" << str << "'" << std::endl;
                MTF::iterator it = transLookUp.find(str);
                if (it != transLookUp.end())
                {//obtain existing tf from look up table
                    tf_map2anchor = transLookUp.at(str);
                }
                else
                {//create new tf
                    CurrentAnchor = str;
                    transLookUp.insert(MTF::value_type(str, tf_map2anchor));
                }
                free(strBuf);
                //continue to next process in (case Recv_Pose:)
            }
            // TODO: FALL THROUGH?

            case Recv_Pose: 
            { // receive camera position as 4x4 float matrix + floor information
                // receive 4x4 matrix and update tf
                recvMessage(sock, sizeof(float) * 20, (char *)cameraPosition);
                Eigen::Matrix4d anc2cam;
                anc2cam << cameraPosition[0], cameraPosition[4], cameraPosition[8], cameraPosition[12],
                    cameraPosition[1], cameraPosition[5], cameraPosition[9], cameraPosition[13],
                    cameraPosition[2], cameraPosition[6], cameraPosition[10], cameraPosition[14],
                    cameraPosition[3], cameraPosition[7], cameraPosition[11], cameraPosition[15];

                tf_anchor2camera = EigenMat2tf(anc2cam);
                tf_anchor2camera.frame_id_ = std::string("spatialAnchor");
                tf_anchor2camera.child_frame_id_ = std::string("hololens");
                tf_anchor2camera.stamp_ = ros::Time::now();
                // broadcast
                tf_br_.sendTransform(tf_anchor2camera);

                // obtain HoloLens height and floor position
                holoLensHeight = cameraPosition[19];
                toFloor<<cameraPosition[16], cameraPosition[17], cameraPosition[18];

                //compute vector from floor to HoloLens
                //floor2holo is perpendicular to the floor
                tf::Transform transform = tf_map2anchor * tf_hololens2ros *  tf_anchor2camera;
                geometry_msgs::PoseStamped floor2holo;

                floor2holo.header.stamp = ros::Time::now();
                floor2holo.header.frame_id = "map";
                floor2holo.pose.position.x = transform.getOrigin().getX() - toFloor(0);
                floor2holo.pose.position.y = transform.getOrigin().getY() - toFloor(1);
                floor2holo.pose.position.z = transform.getOrigin().getZ() - toFloor(2);

                //rotation (1,0,0) to bestn
                Eigen::Vector3d stdVec, rotAx;
                Eigen::Vector3d bestn = toFloor/(-holoLensHeight);

                stdVec << 1,0,0;
                rotAx = (stdVec.cross(bestn)).normalized();

                double angle = acos(stdVec.dot(bestn));

                floor2holo.pose.orientation.x = rotAx(0) * sin(angle/2);
                floor2holo.pose.orientation.y = rotAx(1) * sin(angle/2);
                floor2holo.pose.orientation.z = rotAx(2) * sin(angle/2);
                floor2holo.pose.orientation.w = cos(angle/2);

                pub2.publish(floor2holo);

                //set new SA's height (does not affect the calculations)
                if (repState == Recv_Anchor_Pose_Map)
                    keptHeight = toFloor(2);
            }
            break;

            default:
                ;
        }

        //broadcast tf
        tf_map2anchor.stamp_ = tf_anchor2camera.stamp_;
        tf_br_.sendTransform(tf_map2anchor);
        tf_hololens2ros.stamp_ = ros::Time::now();
        tf_br_.sendTransform(tf_hololens2ros);

        if (transBool)
        {
            //publish point cloud if receivint it from HoloLens
            pointcloud.header.frame_id = "spatialAnchor";
            pointcloud.header.stamp = ros::Time::now();
            pub.publish(pointcloud);
            transBool = false;
        }

        ros::spinOnce();
    }

    ROS_INFO("Exiting...");

    return 0;
}
