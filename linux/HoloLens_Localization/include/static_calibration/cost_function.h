
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

#ifndef NONLINEAR_SOLVER
#define NONLINEAR_SOLVER

Eigen::Matrix3d axisRot2R(double rx, double ry, double rz);
void R2axisRot(Eigen::Matrix3d R,double& rx,double& ry,double& rz);

struct CostFunctor
{
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

//e=|AX-XB|
struct simple_costfunctor
{
public:
  simple_costfunctor(Eigen::Matrix4d& A_,Eigen::Matrix4d& B_){A=A_;B=B_;};
  bool operator()(const double* parameters, double* residual)const{
    double tx=parameters[0];
    double ty=parameters[1];
    double tz=parameters[2];
    double rx=parameters[3];
    double ry=parameters[4];
    double rz=parameters[5];

    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);
    Eigen::Matrix4d X;
    X.block(0,0,3,3)=R;
    X(0,3)=tx;
    X(1,3)=ty;
    X(2,3)=tz;
    X.block(3,0,1,4)<<0,0,0,1;
    //std::cout<<X<<"\n";
    Eigen::Matrix4d E=A*X-X*B;
    //std::cout<<E<<"\n";
    residual[0]=E(0,0);    residual[1]=E(0,1);    residual[2]=E(0,2);
    residual[3]=E(1,0);    residual[4]=E(1,1);    residual[5]=E(1,2);
    residual[6]=E(2,0);    residual[7]=E(2,1);    residual[8]=E(2,2);
    residual[9]=E(0,3);    residual[10]=E(1,3);    residual[11]=E(2,3);    
    return true;
  }

private:
  Eigen::Matrix4d A,B;
};


//v1=Rv2 style
struct F1{
public:
  F1(Eigen::Vector3d& ka_,Eigen::Vector3d& kb_){ka=ka_;kb=kb_;};
  bool operator()(const double* parameters, double* residual)const{
    double tx=parameters[0];
    double ty=parameters[1];
    double tz=parameters[2];
    double rx=parameters[3];
    double ry=parameters[4];
    double rz=parameters[5];

    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);
    Eigen::Vector3d kbd=R*kb;
    //std::cout<<X<<"\n";
    //std::cout<<E<<"\n";
    residual[0]=(ka.cross(kbd)).norm();
    return true;
  }

private:
  Eigen::Vector3d ka,kb;
};

//Ra*tx+ta=Rx*tb+tx
struct F2{
public:
  F2(Eigen::Matrix4d& A_,Eigen::Matrix4d& B_){A=A_;B=B_;};
  bool operator()(const double* parameters, double* residual)const{
    double tx=parameters[0];
    double ty=parameters[1];
    double tz=parameters[2];
    double rx=parameters[3];
    double ry=parameters[4];
    double rz=parameters[5];
    
    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);    
    Eigen::Matrix3d Ra=A.block(0,0,3,3);
    
    Eigen::Vector3d t,ta,tb,leftv,rightv;t<<tx,ty,tz;
    ta=A.block(0,3,3,1);
    tb=B.block(0,3,3,1);
    leftv=Ra*t+ta;
    rightv=R*tb+t;
    
    Eigen::Vector3d cp=leftv.cross(rightv);
    residual[0]=cp.norm()/(leftv.norm()*rightv.norm());//sin(theta)    
    return true;
  }

private:
  Eigen::Matrix4d A,B;
};

//obtain height parameter
struct F3{
public:
  F3(Eigen::Vector3d& floor2holo_,Eigen::Vector3d& head2foot_){floor2holo=floor2holo_;head2foot=head2foot_;};
  bool operator()(const double* parameters, double* residual)const{
    double tx=parameters[0];
    double ty=parameters[1];
    double tz=parameters[2];
    double rx=parameters[3];
    double ry=parameters[4];
    double rz=parameters[5];
    
    
    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);    
    Eigen::Vector3d t;t<<tx,ty,tz;
    
    Eigen::Vector3d o=floor2holo-R.transpose()*t+R.transpose()*head2foot;
        Eigen::Vector3d nf2h=floor2holo.normalized();
	Eigen::Vector3d no=o.normalized();
	
    residual[0]=floor2holo.dot(o);    
    return true;
  }

private:
  Eigen::Vector3d floor2holo,head2foot;
};

//v1=Rv2 style
struct F1_{
public:
  F1_(Eigen::Vector3d& ka_,Eigen::Vector3d& kb_){ka=ka_;kb=kb_;};
  bool operator()(const double* parameters, double* residual)const{
    double rx=parameters[0];
    double ry=parameters[1];
    double rz=parameters[2];

    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);
    Eigen::Vector3d kbd=R*kb;
    //std::cout<<X<<"\n";
    //std::cout<<E<<"\n";
    residual[0]=(ka.cross(kbd)).norm();
    return true;
  }

private:
  Eigen::Vector3d ka,kb;
};

//Ra*tx+ta=Rx*tb+tx
struct F2_{
public:
  F2_(Eigen::Matrix4d& A_,Eigen::Matrix4d& B_,double* rotParam_){A=A_;B=B_;rotParam=rotParam_;};
  bool operator()(const double* parameters, double* residual)const{
    double tx=parameters[0];
    double ty=parameters[1];
    double tz=parameters[2];
    double rx=rotParam[0];
    double ry=rotParam[1];
    double rz=rotParam[2];
    
    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);    
    Eigen::Matrix3d Ra=A.block(0,0,3,3);
    
    Eigen::Vector3d t,ta,tb,leftv,rightv;t<<tx,ty,tz;
    ta=A.block(0,3,3,1);
    tb=B.block(0,3,3,1);
    leftv=Ra*t+ta;
    rightv=R*tb+t;
    
    Eigen::Vector3d cp=leftv.cross(rightv);
    residual[0]=cp.norm()/(leftv.norm()*rightv.norm());//sin(theta)    
    return true;
  }

private:
  Eigen::Matrix4d A,B;
  double* rotParam;
};

//obtain height parameter
struct F3_{
public:
  F3_(Eigen::Vector3d& floor2holo_,Eigen::Vector3d& head2foot_,double* rotParam_){floor2holo=floor2holo_;head2foot=head2foot_;rotParam=rotParam_;};
  bool operator()(const double* parameters, double* residual)const{
    double tx=parameters[0];
    double ty=parameters[1];
    double tz=parameters[2];
    double rx=rotParam[0];
    double ry=rotParam[1];
    double rz=rotParam[2];
    
    
    Eigen::Matrix3d R=axisRot2R(rx,ry,rz);    
    Eigen::Vector3d t;t<<tx,ty,tz;
    
    Eigen::Vector3d o=floor2holo-R.transpose()*t+R.transpose()*head2foot;
        Eigen::Vector3d nf2h=floor2holo.normalized();
	Eigen::Vector3d no=o.normalized();
	
    residual[0]=floor2holo.dot(o);    
    return true;
  }

private:
  Eigen::Vector3d floor2holo,head2foot;
  double* rotParam;
};


#endif
