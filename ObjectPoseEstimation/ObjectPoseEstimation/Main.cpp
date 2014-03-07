/**
 * \file Main.cpp
 * \brief Main entry point for Object Pose Estimation
 * \author Kester Duncan
 *
 * This file is the main entry point for using the object pose estimator using superquadrics by 
 * Kester Duncan
 */
#if 1
#include <iostream>
#include "SQTypes.h"
#include "ObjectPoseEstimator.h"

using namespace std;

int main (int argc, char *argv[]) {
	ope::OPESettings settings; // Using default settings
	settings.minTgtDepth = 0.4f;
	settings.maxObjHeight = 2.5f;

	ope::ObjectPoseEstimator poseEstimator;
	//poseEstimator.init(settings);

	ope::SQParameters sqParams = ope::ObjectPoseEstimator::run(settings);
	ope::verbose = false;
	
	cin.get();
	cin.get();

	return 0;
}


#endif