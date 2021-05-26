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

#include "ICP_module.h"

const char * pszAppName = "hololens_anchor_localizer";

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
ICP2D_Module::ICP2D_Module(ros::NodeHandle n)
{
    sub = n.subscribe<sensor_msgs::PointCloud>("/hololens/pc",1000,&ICP2D_Module::callbackPC,this);
    submap = n.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &ICP2D_Module::callbackMap, this);

    subinitpose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &ICP2D_Module::callbackInitPose, this);

    pub = n.advertise<sensor_msgs::PointCloud>("/hololens/pc2d",1000);
    pubpre = n.advertise<sensor_msgs::PointCloud>("/hololens/pc2d_pre",1000);
    pubimg = n.advertise<sensor_msgs::Image>("/hololens/image",1000);
    pubreq = n.advertise<std_msgs::Bool>("/hololens/ack",1000);
    pubarr = n.advertise<geometry_msgs::PoseStamped>("hololens/localized",1000);

    //plane surface normal direction for cross section computation
    nv << 0,1,0;
    h = 0;

    t1 << 1,0,0;
    t2 << 0,0,1;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void ICP2D_Module::callbackInitPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initPose)
{
    //
    // subscribe initial pose on Rviz
    //

    // convert initial pose to 2D position and direction angle
    initx = initPose->pose.pose.position.x;
    inity = initPose->pose.pose.position.y;

    tf::Quaternion initq(
        initPose->pose.pose.orientation.x,
        initPose->pose.pose.orientation.y,
        initPose->pose.pose.orientation.z,
        initPose->pose.pose.orientation.w);

    tf::Matrix3x3 mat(initq);

    yaw = acos(mat[0][0]);

    if (mat[0][1] > 0)
        yaw=-yaw;

    ROS_INFO("2D Pose Estimate request received. Requesting to initialize HoloLens bridge...");

    // request initialize to bridge
    std_msgs::Bool req;

    req.data = true;

    pubreq.publish(req);

    bInit = true;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void ICP2D_Module::callbackPC(const sensor_msgs::PointCloud::ConstPtr& pointcloud)
{
    ROS_INFO("HoloLens point cloud data received. Calculating optimal rotation and transform...");

    // point cloud: SA coordinates
    sensor_msgs::PointCloud pc2d,pc2d_prev;
    double err;
    geometry_msgs::Point32 pt2;
    std::vector<Eigen::Vector2d> querypoints;

    // 3D to 2D :SA coordinates to ros coordinates
    double range[4]={100,-100,100,-100};//minx,maxx,miny,maxy to determine map size
    for (int i = 0; i < pointcloud->points.size(); i++) {
        geometry_msgs::Point32 pt = pointcloud->points.at(i);

        distFromPlane(pt,pt2,err);//compute point2plane distance

        if (err < 0.1) {
            Eigen::Vector2d qp;qp<<-pt2.z,-pt2.x;
            querypoints.push_back(qp);      
            if (range[0] > qp(0))
                range[0] = qp(0);
            if (range[1] < qp(0))
                range[1] = qp(0);
            if (range[2] > qp(1))
                range[2] = qp(1);
            if (range[3] < qp(1))
                range[3] = qp(1);      
        }
    }

    //std::cout << range[0] <<"," << range[1] << "," << range[2] <<"," << range[3] <<std::endl;

    // create image for building pre-map
    sensor_msgs::Image img;
    double pixresolution = 0.01;//0.01 m/pix
    int margen = 40;
    img.header.frame_id = "map";
    img.width = (range[1]-range[0])/pixresolution+margen*2;
    img.height = (range[3]-range[2])/pixresolution+margen*2;
    img.encoding = sensor_msgs::image_encodings::MONO8;
    img.step = img.width;
    img.data = std::vector<unsigned char>(img.width*img.height, 255);

    //initial pose for alignment
    //given from Rviz(first time) or tf
    if (!bInit) {
        // obtain tf between map-SA(ROS)
        try {
     	    tf::StampedTransform transform;
            listener.waitForTransform("/map", "/spatialAnchor_ros", pointcloud->header.stamp, ros::Duration(5.0));
            listener.lookupTransform(std::string("map"),std::string("/spatialAnchor_ros"),
            pointcloud->header.stamp,transform);
            initx = transform.getOrigin().getX();
            inity = transform.getOrigin().getY(); 
            tf::Matrix3x3 mat = transform.getBasis();
            yaw = acos(mat[0][0]);	    
            if (mat[0][1] > 0)
                yaw = -yaw;   
        } catch (tf::TransformException ex) {
  	        ROS_ERROR("listener error!!");
  	    }    
    }

    bInit=false;
    // 2D(x,y,yaw)-->2D R,t
    tf::Quaternion initq;
    Eigen::Matrix2d R;
    Eigen::Vector2d T;

    R << cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
    T << initx,inity;

    std::cout << pszAppName << ": rotation matrix" << std::endl;
    std::cout << /* std::setw(16) << */ R << std::endl;
    std::cout << pszAppName << ": transform" << std::endl;
    std::cout << /* std::setw(16) << */ T << std::endl;
    std::cout << pszAppName << ": query point size: " << querypoints.size() << std::endl;
  
    //build pre-map + point cloud before alignment(for displaying)
    for (int i = 0; i < querypoints.size(); i++) {
	    Eigen::Vector2d pt;

        pt << querypoints.at(i)(0),querypoints.at(i)(1);

        //compute correspond pixel coordinates
        int plotx = img.width -((pt(0)-range[0])/pixresolution+margen);
        int ploty = (pt(1)-range[2])/pixresolution+margen;
        int idx = ploty * img.step + plotx;
        img.data[idx] = 0; // fill pixel as black

        pt = R*pt + T;

        geometry_msgs::Point32 gp;

        gp.x = pt(0);
        gp.y = pt(1);
        gp.z = 0;

        pc2d_prev.points.push_back(gp);
	}

    pc2d_prev.header.frame_id = "map";

    // publish image and point cloud
    pubpre.publish(pc2d_prev);
    pubimg.publish(img);

    if (rtree.size() == 0)
        return; // return if pre-map is not published   

    {     //cvt initial 2D rotation to 3D quaternion
        tf::Matrix3x3 mat(R(0,0),R(0,1),0,
				R(1,0),R(1,1),0,
				0,0,1);

	    mat.getRotation(initq);
    }

    // registration (simple 2d ICP alignment)
    bool positionLost = false;
	for (int itr = 0; itr < 10; itr++) {
	    std::vector<int> idx;
	    std::vector<Eigen::Vector2d> dstpoints,querypoints_opt;

	    if (querypoints.size() < 50) {
            ROS_ERROR("data shortage!!");
		    positionLost = true;
	        break;
	    }

        // searching nearest points
	    // printf("%s: searching nearest points...\n", pszAppName);
	    for (int i = 0; i < querypoints.size();) {
	        std::vector<value> result_n;
            Eigen::Vector2d qp;
            
            qp << querypoints[i](0),querypoints[i](1);

            //transform query point (HoloLens)
            qp = R*qp + T;

            // search nearest point
	        rtree.query(bgi::nearest(point(qp(0),qp(1)),1),std::back_inserter(result_n));
        
            // nearest point in pre-map
	        float x = result_n[0].first.get<0>();
	        float y = result_n[0].first.get<1>();
	    
	        Eigen::Vector2d dp;
            
            dp << x,y;

            if ((qp - dp).norm() > 1.0) {
                i++;
                continue; //outlier rejection (constant threshold: 1m)
            }

            querypoints_opt.push_back(querypoints[i]);
	        dstpoints.push_back(dp);
	        i++;
	    }

	    // printf("%s: finding optimal rotation and transform...\n", pszAppName);

        // obtain optimal R,t (solve ICP)
	    linearCompute(querypoints_opt,dstpoints,R,T);

	    yaw = acos(R(0,0));
	    if (R(0,1) > 0)
            yaw = -yaw;

        // cvt 2D rotation to 3D quaternion
	    tf::Matrix3x3 mat(R(0,0),R(0,1),0,
				R(1,0),R(1,1),0,
				0,0,1
			    );

	    mat.getRotation(initq);

	    initx = T(0);
	    inity = T(1); 

        // std::cout << pszAppName << ": " << R << std::endl << initx << "," << inity << "," << yaw << std::endl;
	}

	ROS_INFO("Calculating optimal rotation and transform completed: x=%.4f, y=%.4f, yaw=%.4f", (float)initx, (float)inity, (float)yaw);

    // publish optimized position and direction
    geometry_msgs::Quaternion  msgqt;

    tf::quaternionTFToMsg(initq, msgqt);

    geometry_msgs::PoseStamped ps;

    ps.pose.position.x = initx;
    ps.pose.position.y = inity;
    ps.pose.position.z = 0;
    ps.pose.orientation = msgqt;
    ps.header.frame_id = "map";

    pubarr.publish(ps);

    // publish aligned 2D point cloud (for displaying)
    for (int i = 0; i < querypoints.size();) {
        std::vector<value> result_n;
        Eigen::Vector2d qp;

        qp << querypoints[i](0),querypoints[i](1);
        qp = R*qp + T;

        geometry_msgs::Point32 gp;

        gp.x = qp(0);
        gp.y = qp(1);
        gp.z = 0;

        pc2d.points.push_back(gp);
        i++;
	}

    pc2d.header.frame_id = "map";

    pub.publish(pc2d);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void ICP2D_Module::callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& subscribedmap)
{
    //
    // get floormap and create rtree
    //

    if (rtree.size() > 0)
        return; // process only once

    int width = subscribedmap->info.width;
    int height = subscribedmap->info.height;
    double scale = subscribedmap->info.resolution;
    unsigned int cnt = 0;

    // cvt pixel to point cloud
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if (subscribedmap->data[i+j*width] > 50) {
                point p = point(i*scale+subscribedmap->info.origin.position.x,j*scale+subscribedmap->info.origin.position.y);
                rtree.insert(std::make_pair(p, ++cnt));
            }
        }
    }

    ROS_INFO("map_server node initialized. Map size is %d x %d pixels. Tree size is %u points.", width, height, cnt);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void ICP2D_Module::distFromPlane(geometry_msgs::Point32 pt,geometry_msgs::Point32& ptret,double& dist)
{
    //
    // compute point to plane distance
    //
    dist = pt.x*nv(0)+pt.y*nv(1)+pt.z*nv(2)-h;
    ptret.x = pt.x-dist*nv(0);
    ptret.y = pt.y-dist*nv(1);
    ptret.z = pt.z-dist*nv(2);
    dist = std::fabs(dist);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void ICP2D_Module::start()
{
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();    
    }
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void ICP2D_Module::linearCompute(std::vector<Eigen::Vector2d> a,std::vector<Eigen::Vector2d> b,Eigen::Matrix2d& R,Eigen::Vector2d& T)
{
    //
    // solve ICP problem
    //

    // get centroid  
    Eigen::Vector2d ga,gb;
    
    ga << 0,0;
    gb << 0,0;
    for (int i = 0; i < a.size(); i++) {
        ga += a.at(i);
        gb += b.at(i);
    }

    ga = ga*(1.0/a.size());
    gb = gb*(1.0/a.size());

    //
    Eigen::Vector2d vga,vgb;

    vga << ga(0),ga(1);
    vgb << gb(0),gb(1);

    Eigen::MatrixXd H(2,2);
    H.setZero();

    // std::cout << pszAppName << ": " << ga.transpose() << ":" << gb.transpose() << std::endl;

    for (int i = 0; i < a.size(); i++) {
        Eigen::Vector2d pa,pb;

        pa << a.at(i)(0),a.at(i)(1);
        pb << b.at(i)(0),b.at(i)(1);

        Eigen::MatrixXd h_=(pa-vga)*(pb-vgb).transpose();

        H=H+h_;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix2d Hm;
    Eigen::Matrix2d uvt = svd.matrixU()*svd.matrixV().transpose();

    Hm << 1,0,0,uvt(0,0)*uvt(1,1)-uvt(0,1)*uvt(1,0);

    R = svd.matrixV()*Hm*svd.matrixU().transpose();

/*  if(R.determinant()<0){

        R.col(2)=-1*R.col(2);    

    }*/

    T = -R*vga+vgb;

    return;
}
