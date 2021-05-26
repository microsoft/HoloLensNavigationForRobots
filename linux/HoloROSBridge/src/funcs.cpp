#include "funcs.h"

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
tf::StampedTransform positionMat2tf(float* posdata)
{
    tf::StampedTransform tf_ret;

    tf_ret.setOrigin(tf::Vector3(posdata[12], posdata[13], posdata[14]));

	tf::Matrix3x3 rot(posdata[0],posdata[4],posdata[8],
		posdata[1],posdata[5],posdata[9],
		posdata[2],posdata[6],posdata[10]);
	tf::Quaternion q;
	rot.getRotation(q);
	tf_ret.setRotation(q);
	tf_ret.stamp_=ros::Time::now();

	return tf_ret;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
tf::StampedTransform EigenMat2tf(Eigen::Matrix4d posdataMat)
{
    double* posdata = posdataMat.data();
    tf::StampedTransform tf_ret;
  
    tf_ret.setOrigin(tf::Vector3(posdata[12], posdata[13], posdata[14]));
	
    tf::Matrix3x3 rot(posdata[0],posdata[4],posdata[8],
		posdata[1],posdata[5],posdata[9],
		posdata[2],posdata[6],posdata[10]);
	tf::Quaternion q;
	rot.getRotation(q);
	tf_ret.setRotation(q);
	tf_ret.stamp_=ros::Time::now();
	  
	return tf_ret;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
Eigen::Matrix4d tf2EigenMat(tf::StampedTransform tfdata)
{
    tf::Vector3 origin = tfdata.getOrigin();
    tf::Matrix3x3 rot =  tfdata.getBasis();
    Eigen::Matrix4d ret;
    ret <<  rot[0][0],rot[0][1],rot[0][2],origin[0],
            rot[1][0],rot[1][1],rot[1][2],origin[1],
            rot[2][0],rot[2][1],rot[2][2],origin[2],
            0,0,0,1;

    return ret;
}
