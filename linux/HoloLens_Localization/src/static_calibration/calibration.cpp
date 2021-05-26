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

#include "calibration.h"

void printHelp();

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
calibrator::calibrator(std::string holoLinkedFrame, std::string odomFrame, std::string robotFootFrame, std::string fpath)
{
    //
    // constructor with file path

	m_holoLinkedFrame = holoLinkedFrame;
	m_odomFrame = odomFrame;
	m_robotFootFrame = robotFootFrame;
	m_fpath = fpath;

    //
    // initialize transform
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    T << 0, 0, 0;

    if (!m_fpath.empty()) {
        std::ifstream ifs(m_fpath, std::ios::binary);
        if (ifs)
        {
            //
            // file exists, read data from it
            float dat[12];
            ifs.read((char *)dat, sizeof(float) * 12);
            R << dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7], dat[8];
            T << dat[9], dat[10], dat[11];
            ifs.close();

            std::cout << "Calibration file '" << m_fpath << "' loaded." << std::endl;
        }
        else
        {
            //
            // file does not exist, write initial data
            writeCalibrationData();
        }
    }

    // set transformation
    setTransform();

	m_horizontalCalibMode = false;
	boost::thread *thr = new boost::thread(boost::bind(&calibrator::publish_thread, this));
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::setTransform()
{
	m_tf_map_to_odom.frame_id_ = std::string(m_holoLinkedFrame);
	m_tf_map_to_odom.child_frame_id_ = std::string("hololens_p");
	m_tf_map_to_odom.setOrigin(tf::Vector3(T(0), T(1), T(2)));

	tf::Matrix3x3 rot(R(0, 0), R(0, 1), R(0, 2),
					  R(1, 0), R(1, 1), R(1, 2),
					  R(2, 0), R(2, 1), R(2, 2));
	tf::Quaternion q;
	rot.getRotation(q);
	m_tf_map_to_odom.setRotation(q);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::poseStampedCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	m_latestPoseStamped = *msg;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::run()
{
	m_holo_floor_sub = m_nh.subscribe < geometry_msgs::PoseStamped>("/holo/floor", 1, &calibrator::poseStampedCB, this);

	std::cout << "Calibration: ready" << std::endl;

	while (ros::ok())
	{
		int c;
        bool fSuccess;

		//get keyboard input
		if (kbhit())
		{
			std::cout << "\r \r";
			c = getch();
		}
		else
		{
			ros::spinOnce();
			continue;
		}

        fSuccess = lookupTransform();
        if (!fSuccess)
            continue;

		if (c == ' ') // space key: record current position
		{
            recordCurrentPosition();
		}
		else if (c == 'c') // calibration
		{
            calibrate();
		}
		else if (c == 'a') // auto-calibration
		{
			autocalibrate();
		}
		else if (c == 'q') // quit
		{
			break;
		}
		else if (c == 'w') // save calibration result
		{
			writeCalibrationData();
		}
		else if (c == 'd') // delete recorded position
		{
			if (m_pep_pos.size() >= 1)
			{
				m_pep_pos.pop_back();
				m_hol_pos.pop_back();
			}
		}
		else if (c == 'z') // save to log
		{
			std::ofstream ofs(m_fpath + ".log");
			std::cout << "Param R" << std::endl;
			std::cout << R << std::endl;
			std::cout << "Param t" << std::endl;
			std::cout << T << std::endl;
			ofs << "Param R" << std::endl;
			ofs << R << std::endl;
			ofs << "Param t" << std::endl;
			ofs << T << std::endl;
		}
		else if (c == 't') // toggle calibration mode (General hand-eye calibration <-> Calibration with limited movements (Using Horizontal movement))
		{
			m_horizontalCalibMode = !m_horizontalCalibMode;

			if (m_horizontalCalibMode)
				std::cout << "horizontal calibration mode: Horizontal" << std::endl;
            else
				std::cout << "horizontal calibration mode: Bi-directional rotation" << std::endl;

			//reset
			m_pep_pos.clear();
			m_hol_pos.clear();
			m_floor2holo.clear();
			m_head2foot.clear();
		}
        else if ((c == '?') || (c == 'h'))
        {
            printHelp();            
        }
		else // tele-operation
		{
			m_tele.operation(c);
		}
	}
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::publish_thread()
{
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		// broadcast transform
		m_tf_map_to_odom.stamp_ = ros::Time::now();
		m_tf_br.sendTransform(m_tf_map_to_odom);

		loop_rate.sleep();
	}
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
bool calibrator::lookupTransform()
{
    try
    {
        //
        // get Odom-holoLinkedFrame, SA-HoloLens transformation
        m_listener.lookupTransform(std::string(m_odomFrame), std::string(m_holoLinkedFrame), ros::Time(0), m_transform1);
        m_listener.lookupTransform(std::string("spatialAnchor"), std::string("hololens"), ros::Time(0), m_transform2);
    }
    catch (tf::TransformException ex)
    {
        std::cout << "Calibration: tf listener error!! " << ex.what() << std::endl;
        return false;
    }

    return true;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::recordCurrentPosition()
{
    if (m_horizontalCalibMode)
    {
        try
        { // obtain map-hololens, Head-base
            m_listener.waitForTransform("/map", "/hololens", m_latestPoseStamped.header.stamp, ros::Duration(5.0));
            m_listener.lookupTransform(std::string("map"), std::string("hololens"), m_latestPoseStamped.header.stamp, m_transform3);
            m_listener.lookupTransform(std::string(m_holoLinkedFrame), std::string(m_robotFootFrame), m_latestPoseStamped.header.stamp, m_transform4);
        }
        catch (tf::TransformException ex)
        {
            std::cout << "Calibration: tf listener error!! " << ex.what() << std::endl;
            return;
        }

        Eigen::Vector3d floor2holoVec, head2footVec;
        // obtain floor to HoloLens vector (map to HoloLens coordinate)
        floor2holoVec << m_transform3.getOrigin().getX() - m_latestPoseStamped.pose.position.x, m_transform3.getOrigin().getY() - m_latestPoseStamped.pose.position.y, m_transform3.getOrigin().getZ() - m_latestPoseStamped.pose.position.z;
        Matrix4d map2holo = btTrans2EigMat4d(m_transform3);
        floor2holoVec = map2holo.block(0, 0, 3, 3).inverse() * floor2holoVec;
        m_floor2holo.push_back(floor2holoVec);

        head2footVec << m_transform4.getOrigin().getX(), m_transform4.getOrigin().getY(), m_transform4.getOrigin().getZ();
        m_head2foot.push_back(head2footVec);
    }

    // Odom-holoLinkedFrame (Robot)
    m_pep_pos.push_back(m_transform1);

    // SA-HoloLens
    m_hol_pos.push_back(m_transform2);

    // display position
    tf::Quaternion q = m_transform1.getRotation();
    std::cout << "Calibration: count " << m_pep_pos.size() << std::endl;
    std::cout << "Calibration: " << m_holoLinkedFrame << ":(" << m_transform1.getOrigin().getX() << "," << m_transform1.getOrigin().getY() << "," << m_transform1.getOrigin().getZ() << "),(" << q.getX() << "," << q.getY() << "," << q.getZ() << "," << q.getW() << ")" << std::endl;
    q = m_transform2.getRotation();
    std::cout << "Calibration: HoloLens:(" << m_transform2.getOrigin().getX() << "," << m_transform2.getOrigin().getY() << "," << m_transform2.getOrigin().getZ() << "),(" << q.getX() << "," << q.getY() << "," << q.getZ() << "," << q.getW() << ")" << std::endl;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::calibrate()
{
    if (m_pep_pos.size() < 3)
    {
        std::cout << "Calibration Error!! The number of recorded positions must be >=3 !!" << std::endl;
        return;
    }

    std::cout << "======= calibration start ========" << std::endl;
    if (!m_horizontalCalibMode)
    {
        calibration(m_pep_pos, m_hol_pos, R, T);
    }
    else
    {
        horizontalCalibration(m_pep_pos, m_hol_pos, m_floor2holo, m_head2foot, R, T);
    }

    // display calibration result
    std::cout << "======= calibration done =========" << std::endl;
    std::cout << "Calibration results:\n translation" << std::endl
                << T << std::endl
                << "rotation" << std::endl
                << R << std::endl;
    m_tf_map_to_odom.setOrigin(tf::Vector3(T(0), T(1), T(2)));
    tf::Matrix3x3 rot(R(0, 0), R(0, 1), R(0, 2),
                        R(1, 0), R(1, 1), R(1, 2),
                        R(2, 0), R(2, 1), R(2, 2));
    tf::Quaternion q;
    rot.getRotation(q);
    m_tf_map_to_odom.setRotation(q);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
bool calibrator::setHeadPositionAndRecord(float fPitch, float fYaw)
{
    bool fSuccess;
	double duration = 3.0;

    m_tele.setPepperHeadPitchYaw(fPitch, fYaw);

    ros::Duration(duration).sleep();
    ros::spinOnce();

    fSuccess = lookupTransform();
    if (!fSuccess)
        return false;

    recordCurrentPosition();

    return true;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
bool calibrator::autocalibrate()
{
    bool fSuccess;

    std::cout << "auto-calibrate: step 1 of 5 - resetting pepper head and recording position..." << std::endl;
    if (!setHeadPositionAndRecord(0.0, 0.0))
        return false;

    std::cout << "auto-calibrate: step 2 of 5 - setting pepper head position A and recording position..." << std::endl;
    if (!setHeadPositionAndRecord(-0.35, 0.0))
        return false;

    std::cout << "auto-calibrate: step 3 of 5 - setting pepper head position B and recording position..." << std::endl;
    if (!setHeadPositionAndRecord(0.00, 0.7))
        return false;

    std::cout << "auto-calibrate: step 4 of 5 - setting pepper head position C and recording position..." << std::endl;
    if (!setHeadPositionAndRecord(0.00, -0.7))
        return false;

    std::cout << "auto-calibrate: step 5 of 5 - resetting pepper head and calibrating..." << std::endl;
    m_tele.setPepperHeadPitchYaw(0.0, 0.0);

    ros::spinOnce();

    fSuccess = lookupTransform();
    if (!fSuccess)
        return false;

    calibrate();

    return true;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::writeCalibrationData()
{
    if (m_fpath.empty())
        return;

    std::ofstream ofs(m_fpath);
    float dat[12];

    dat[0] = R(0, 0);
    dat[1] = R(0, 1);
    dat[2] = R(0, 2);
    dat[3] = R(1, 0);
    dat[4] = R(1, 1);
    dat[5] = R(1, 2);
    dat[6] = R(2, 0);
    dat[7] = R(2, 1);
    dat[8] = R(2, 2);
    dat[9] = T(0);
    dat[10] = T(1);
    dat[11] = T(2);

    ofs.write((char *)dat, sizeof(float) * 12);
    ofs.close();

    std::cout << "Calibration file '" << m_fpath << "' saved." << std::endl;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d &R, Vector3d &T)
{
	linear_calibration(pep_pos, hol_pos, R, T);
	std::cout << "======== finished initial parameter computation ========" << std::endl;
	nonlinear_calibration(pep_pos, hol_pos, R, T);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::horizontalCalibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> floor_holo, std::vector<Eigen::Vector3d> head_foot, Matrix3d &R, Vector3d &T)
{
	double bestScores[2];
	horizontal_initialization(pep_pos, hol_pos, floor_holo, head_foot, R, T, bestScores);
	std::cout << "======== finished initial parameter computation ========" << std::endl;
	nonlinear_horizontal_calibration(pep_pos, hol_pos, floor_holo, head_foot, R, T, bestScores);
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::linear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d &R, Vector3d &T)
{
	std::vector<Matrix4d> pepMat, pepdMat;
	std::vector<Matrix4d> holMat, holdMat;

	// obtain motion
	for (int i = 0; i < pep_pos.size(); i++)
	{
		Matrix4d mp = btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh = btTrans2EigMat4d(hol_pos.at(i));
		pepMat.push_back(mp);
		holMat.push_back(mh);
		if (i > 0)
		{
			pepdMat.push_back(pepMat.at(i - 1).inverse() * pepMat.at(i));
			holdMat.push_back(holMat.at(i - 1).inverse() * holMat.at(i));
		}
	}

	// Rotation calibration
	// obtain axis from motion
	std::vector<Vector3d> pepAxis, holAxis;
	MatrixXd KA(3, pepdMat.size()), KB(3, pepdMat.size());
	for (int i = 0; i < pepdMat.size(); i++)
	{
		Vector3d pepax = mat2axis(pepdMat.at(i));
		Vector3d holax = mat2axis(holdMat.at(i));
		KA.col(i) = pepax;
		KB.col(i) = holax;
	}

	MatrixXd KBKA = KB * KA.transpose();
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(KBKA, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d Hm;
	Eigen::Matrix3d uvt = svd.matrixU() * svd.matrixV().transpose();
    
	Hm << 1, 0, 0,
		0, 1, 0,
		0, 0, uvt.determinant();
	R = svd.matrixV() * Hm * svd.matrixU().transpose();

	// Translation calibration
	// solve least square problem for t
	MatrixXd A(pepdMat.size() * 2, 3);
	VectorXd B(pepdMat.size() * 2);

	for (int i = 0; i < pepdMat.size(); i++)
	{
		Vector3d tpep, thol;
		tpep << pepdMat.at(i)(0, 3), pepdMat.at(i)(1, 3), pepdMat.at(i)(2, 3);
		thol << holdMat.at(i)(0, 3), holdMat.at(i)(1, 3), holdMat.at(i)(2, 3);
		Vector3d rightt = tpep - R * thol;
		Matrix3d leftm = Matrix3d::Identity() - pepdMat.at(i).block(0, 0, 3, 3);
		A.block(i * 2, 0, 2, 3) = leftm.block(0, 0, 2, 3);
		B(i * 2) = rightt(0);
		B(i * 2 + 1) = rightt(1);
	}

	T = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
	std::cout << "Rotation (Initial)" << std::endl
			  << R << std::endl;
	std::cout << "Translation (Initial)" << std::endl
			  << T << std::endl;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::nonlinear_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, Matrix3d &R, Vector3d &T)
{
	double opt[6];
	double optrot[3];
	double opttrans[3];

	R2axisRot(R, opt[3], opt[4], opt[5]);
	R2axisRot(R, optrot[0], optrot[1], optrot[2]);

	opt[0] = T(0);
	opt[1] = T(1);
	opt[2] = T(2);

	opttrans[0] = T(0);
	opttrans[1] = T(1);
	opttrans[2] = T(2);

	Problem problem, problem_a, problem_b;

	std::vector<Matrix4d> pepMat, pepdMat;
	std::vector<Matrix4d> holMat, holdMat;

	// Obtain motion
	for (int i = 0; i < pep_pos.size(); i++)
	{
		Matrix4d mp = btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh = btTrans2EigMat4d(hol_pos.at(i));
		pepMat.push_back(mp);
		holMat.push_back(mh);
		if (i > 0)
		{
			pepdMat.push_back(pepMat.at(i - 1).inverse() * pepMat.at(i));
			holdMat.push_back(holMat.at(i - 1).inverse() * holMat.at(i));
		}
	}

	// Construct cost function
	for (int i = 0; i < pepdMat.size(); i++)
	{
		Matrix4d A = pepdMat.at(i);
		Matrix4d B = holdMat.at(i);
		double anglea, angleb;
		Vector3d a_ax;
		mat2axis_angle(A, a_ax, anglea);
		Vector3d b_ax;
		mat2axis_angle(B, b_ax, angleb);
		//Function 1: Optimize rotation
		CostFunction *cost_function1d = new NumericDiffCostFunction<F1_, ceres::CENTRAL, 1, 3>(new F1_(a_ax, b_ax));
		problem_a.AddResidualBlock(cost_function1d, NULL, optrot);
		//Function 2: Optimize translation
		CostFunction *cost_function2d = new NumericDiffCostFunction<F2_, ceres::CENTRAL, 1, 3>(new F2_(A, B, optrot));
		problem_b.AddResidualBlock(cost_function2d, NULL, opttrans);
	}

	Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;

	// Rotation optimization
	ceres::Solve(options, &problem_a, &summary);

	// Translation optimization
	ceres::Solve(options, &problem_b, &summary);

	std::cout << "finished optimization\n";
	std::cout << summary.BriefReport() << "\n";

	R = axisRot2R(optrot[0], optrot[1], optrot[2]);
	T << opttrans[0], opttrans[1], opttrans[2];
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::horizontal_initialization(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> verticalVecs, std::vector<Eigen::Vector3d> normVecs, Matrix3d &R, Vector3d &T, double *bestScores)
{
	// Compute motion
	std::vector<Matrix4d> RobotMat, A_Mat;
	std::vector<Matrix4d> holMat, B_Mat;

	for (int i = 0; i < pep_pos.size(); i++)
	{
		Matrix4d mp = btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh = btTrans2EigMat4d(hol_pos.at(i));
		RobotMat.push_back(mp);
		holMat.push_back(mh);
		if (i > 0)
		{
			A_Mat.push_back(RobotMat.at(i - 1).inverse() * RobotMat.at(i));
			B_Mat.push_back(holMat.at(i - 1).inverse() * holMat.at(i));
		}
	}

	// Rotation calibration
	std::vector<Vector3d> RobotAxis, holAxis;
	std::cout << "axis of rotation matrix" << std::endl;

	// Obtain rotational axis
	for (int i = 0; i < A_Mat.size(); i++)
	{
		double anglea, angleb;
		Vector3d pepax;
		mat2axis_angle(A_Mat.at(i), pepax, anglea);
		Vector3d holax;
		mat2axis_angle(A_Mat.at(i), holax, angleb);
		if (anglea < 0.1 || angleb < 0.1)
			continue; // reject small rotational movement
		RobotAxis.push_back(pepax);
		holAxis.push_back(holax);
	}

	if (RobotAxis.size() == 0)
	{
		std::cout << "initialization failed!! (need large rotaion)\nPlease check how to move..." << std::endl;
		return;
	}

	// obtain forward movement
	double offset_trans = 0.1;
	double offset = 0.01;
	for (int i = 0; i < A_Mat.size(); i++)
	{
		double anglea, angleb;
		Vector3d pepax;
		mat2axis_angle(A_Mat.at(i), pepax, anglea);
		Vector3d holax;
		mat2axis_angle(B_Mat.at(i), holax, angleb);
		Vector3d ta = A_Mat.at(i).block(0, 3, 3, 1);
		double score = ta.norm() / (fabs(anglea) + offset); // (movement length)/(rotational angle)
		if (2 < score) // 2: threshold
		{ 
			RobotAxis.push_back(ta.normalized());
			Vector3d tb = B_Mat.at(i).block(0, 3, 3, 1);
			holAxis.push_back(tb.normalized());
		}
	}

	// Rotation calibration (SVD)
	MatrixXd KA(3, RobotAxis.size()), KB(3, RobotAxis.size());
	for (int i = 0; i < RobotAxis.size(); i++)
	{
		KA.col(i) = RobotAxis.at(i);
		KB.col(i) = holAxis.at(i);
	}

	MatrixXd KBKA = KB * KA.transpose();
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(KBKA, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d Hm;
	Eigen::Matrix3d uvt = svd.matrixU() * svd.matrixV().transpose();

	Hm << 1, 0, 0,
		0, 1, 0,
		0, 0, uvt.determinant();
	R = svd.matrixV() * Hm * svd.matrixU().transpose();

	// Translation calibration
	std::vector<Vector3d> left_vectors;
	std::vector<double> right_values;

	// vertical trans component
	for (int i = 0; i < verticalVecs.size(); i++)
	{
		Eigen::Vector3d floor2holo = verticalVecs.at(i);
		Eigen::Vector3d head2foot = normVecs.at(i);
		double right_value = floor2holo.norm() + floor2holo.dot(R.transpose() * head2foot); //in hololens frame
		Eigen::Vector3d left_vector = (floor2holo.transpose() * R.transpose()).transpose();
		left_vectors.push_back(left_vector);
		right_values.push_back(right_value);
	}

	// horizontal trans component
	for (int i = 0; i < A_Mat.size(); i++)
	{
		double anglea, angleb;
		Vector3d pepax;
		mat2axis_angle(A_Mat.at(i), pepax, anglea);
		Vector3d holax;
		mat2axis_angle(B_Mat.at(i), holax, angleb);
		Vector3d ta = A_Mat.at(i).block(0, 3, 3, 1);
		Vector3d tb = B_Mat.at(i).block(0, 3, 3, 1);
		if (anglea < 0.1 || angleb < 0.1)
			continue;
		Vector3d rightt = ta - R * tb;
		Matrix3d leftM = Matrix3d::Identity() - A_Mat.at(i).block(0, 0, 3, 3);
		for (int j = 0; j < 3; j++)
		{
			right_values.push_back(rightt(j));
			left_vectors.push_back(leftM.row(j).transpose());
		}
	}

	// solve t by SVD
	MatrixXd A(right_values.size(), 3);
	VectorXd B(right_values.size());

	for (int i = 0; i < right_values.size(); i++)
	{
		A.block(i, 0, 1, 3) = left_vectors.at(i).transpose();
		B(i) = right_values.at(i);
	}

	T = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);

	std::cout << "Rotation (Initial)\n"
			  << R << std::endl;
	std::cout << "Translation (Initial)" << std::endl
			  << T << std::endl;
}
/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/
void calibrator::nonlinear_horizontal_calibration(std::vector<tf::StampedTransform> pep_pos, std::vector<tf::StampedTransform> hol_pos, std::vector<Eigen::Vector3d> floor_holo, std::vector<Eigen::Vector3d> head_foot, Matrix3d &R, Vector3d &T, double *bestScores)
{
	//decompose
	double opt[6];
	double optrot[3];
	double opttrans[3];

	R2axisRot(R, opt[3], opt[4], opt[5]);
	R2axisRot(R, optrot[0], optrot[1], optrot[2]);

	opt[0] = T(0);
	opt[1] = T(1);
	opt[2] = T(2);

	opttrans[0] = T(0);
	opttrans[1] = T(1);
	opttrans[2] = T(2);

	// optimize (2 patterns for comparison)
	// problem: simultaneous 6 parameters optimization
	// problem a, b: optimize sepalately (3 rotation, 3 translation) 
	Problem problem, problem_a, problem_b;

	std::vector<Matrix4d> pepMat, pepdMat;
	std::vector<Matrix4d> holMat, holdMat;
	double offset_trans = 0.1;
	double offset_angle = 0.2;
	double offset = 0.01;

	for (int i = 1; i < pep_pos.size(); i++)
	{ // rotation estimation by horizontal rotation
		// obtain motion (mh3,mp3)
		Matrix4d mp = btTrans2EigMat4d(pep_pos.at(i - 1));
		Matrix4d mh = btTrans2EigMat4d(hol_pos.at(i - 1));
		Matrix4d mp2 = btTrans2EigMat4d(pep_pos.at(i));
		Matrix4d mh2 = btTrans2EigMat4d(hol_pos.at(i));
		Matrix4d mh3, mp3;
		mp3 = mp2.inverse() * mp;
		mh3 = mh2.inverse() * mh;
		double anglea, angleb;
		Vector3d pepax;

		mat2axis_angle(mp3, pepax, anglea);

		Vector3d holax;

		mat2axis_angle(mh3, holax, angleb);

		Vector3d ta = mp3.block(0, 3, 3, 1);
		Vector3d tb = mh3.block(0, 3, 3, 1);
		double scorea = (sqrt(ta.norm()) - sqrt(offset_trans)) / (fabs(anglea) + offset); // translation score
		double scoreb = (sqrt(fabs(anglea)) - sqrt(offset_angle)) / (ta.norm() + offset); // rotation score

		if (scoreb >= bestScores[1] * 0.95) // straight movement
		{
			CostFunction *cost_function1 = new NumericDiffCostFunction<F1, ceres::CENTRAL, 1, 6>(new F1(pepax, holax));
			problem.AddResidualBlock(cost_function1, NULL, opt);
			CostFunction *cost_function2 = new NumericDiffCostFunction<F2, ceres::CENTRAL, 1, 6>(new F2(mp3, mh3));
			problem.AddResidualBlock(cost_function2, NULL, opt);

			CostFunction *cost_function1d = new NumericDiffCostFunction<F1_, ceres::CENTRAL, 1, 3>(new F1_(pepax, holax));
			problem_a.AddResidualBlock(cost_function1d, NULL, optrot);
			CostFunction *cost_function2d = new NumericDiffCostFunction<F2_, ceres::CENTRAL, 1, 3>(new F2_(mp3, mh3, optrot));
			problem_b.AddResidualBlock(cost_function2d, NULL, opttrans);
		}

		if (scorea >= bestScores[0] * 0.95) // rotational movement
		{
			Vector3d ta_n = ta.normalized();
			Vector3d tb_n = tb.normalized();
			CostFunction *cost_function1 = new NumericDiffCostFunction<F1, ceres::CENTRAL, 1, 6>(new F1(ta_n, tb_n));
			problem.AddResidualBlock(cost_function1, NULL, opt);
			CostFunction *cost_function1d = new NumericDiffCostFunction<F1_, ceres::CENTRAL, 1, 3>(new F1_(ta_n, tb_n));
			problem_a.AddResidualBlock(cost_function1d, NULL, optrot);
		}
	}

	for (int i = 0; i < floor_holo.size(); i++)
	{ // height information
		Eigen::Vector3d fh = floor_holo.at(i);
		Eigen::Vector3d hf = head_foot.at(i);
		CostFunction *cost_function3 = new NumericDiffCostFunction<F3, ceres::CENTRAL, 1, 6>(new F3(fh, hf));
		problem.AddResidualBlock(cost_function3, NULL, opt);
		CostFunction *cost_function3d = new NumericDiffCostFunction<F3_, ceres::CENTRAL, 1, 3>(new F3_(fh, hf, optrot));
		problem_b.AddResidualBlock(cost_function3d, NULL, opttrans);
	}

	Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary); // pattern 1 
	ceres::Solve(options, &problem_a, &summary); // pattern 2 a (rotation)
	ceres::Solve(options, &problem_b, &summary); // pattern 2 b (translation)

	std::cout << "finished optimization\n";
	std::cout << summary.BriefReport() << "\n";

	R = axisRot2R(opt[3], opt[4], opt[5]);
	T << opt[0], opt[1], opt[2];

    // pattern 1 result (just for comparison)
	std::cout << R << std::endl;
	std::cout << T << std::endl;

	R = axisRot2R(optrot[0], optrot[1], optrot[2]);
	T << opttrans[0], opttrans[1], opttrans[2];

    // pattern 2 result (used as a calibration result)
	std::cout << R << std::endl;
	std::cout << T << std::endl;
}
