#include "ros/ros.h"

#include <iostream>
#include <iomanip>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <boost/geometry/index/rtree.hpp>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<point, unsigned> value;

class ICP2D_Module
{
public:
    ICP2D_Module(ros::NodeHandle n);
    void start();

private:
    void callbackPC(const sensor_msgs::PointCloud::ConstPtr& pointcloud);
    void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& subscribedmap);
    void callbackInitPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initPose);

    void distFromPlane(geometry_msgs::Point32 pt,geometry_msgs::Point32& ptret,double& dist);
    void linearCompute(std::vector<Eigen::Vector2d> ,std::vector<Eigen::Vector2d>,Eigen::Matrix2d& R,Eigen::Vector2d& T);

#include <iomanip>
    Eigen::Vector3d         nv, t1, t2;
    double                  h; // plane parameter
    ros::Publisher          pub, pubpre, pubimg, pubreq, pubarr;
    ros::Subscriber         sub, submap, subinitpose;
    tf::TransformListener   listener;
    bgi::rtree< value, bgi::quadratic<16> > rtree; 
    double                  initx, inity, yaw; 
    bool                    bInit;
};
