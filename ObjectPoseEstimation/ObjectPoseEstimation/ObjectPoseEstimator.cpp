#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include "SQTypes.h"
#include "Common.h"
#include "ObjectPoseEstimator.h"
#include "Utils.h"
#include "Minimization.h"
#include "SQFitting.h"
#include "PointCloudCapture.h"
#include "TableObjectModeler.h"

using namespace std;
using namespace pcl;


namespace ope {


void ObjectPoseEstimator::init(const OPESettings& opeSettings) {
	ope::minimumTgtDepth = opeSettings.minTgtDepth;
	ope::maximumTgtDepth = opeSettings.maxTgtDepth;
	ope::minimumObjHeight = opeSettings.minObjHeight;
	ope::maximumObjHeight = opeSettings.maxObjHeight;
	ope::verbose = opeSettings.verbose;
	ope::minimumIterations = opeSettings.minIterations;
	ope::allowTapering = opeSettings.allowTapering;
	ope::objectVoxelSize = opeSettings.objVoxelSize;
	ope::doDebug = opeSettings.allowDebug;

}


SQParameters ObjectPoseEstimator::calculateObjectPose(pcl::PointCloud<pcl::PointXYZRGB>& selectedObjectPtCloud) {
	// The initial XYZ point cloud that is processes for pose estimation
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// The downsampled XYZ point cloud that is used in the multiscale voxelization scheme
	pcl::PointCloud<pcl::PointXYZ> downsampledCloud;

	// A pointer to the initial XYZ point cloud used during voxelization
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>());

	// Initial superquadric parameters based on the cloud properties
	SQParameters initParams;	

	// The final (best) superquadric parameters that are found
	SQParameters bestParams;

	// Estimate of the superquadric parameters used for bootstrapping the multiscale voxelization scheme
	SQParameters sqEstimate;

	// A record of the initial number of points before repeated downsampling
	int numberOfPoints = 0;

	/*
	 * The point cloud must be transformed from the Kinect optical frame to the
	 * world coordinate frame. Additional transformations may have to be done depending
	 * on the application, but this is left up to the user.
	 */
	Utils::transformPointCloud(selectedObjectPtCloud);

	/*
	 * The captured XYZRGB point cloud must be converted to an XYZ cloud
	 * in order to perform pose estimation
	 */
	for (size_t i = 0; i < selectedObjectPtCloud.size(); ++i) {
		pcl::PointXYZ p;
		p.x = selectedObjectPtCloud[i].x;
		p.y = selectedObjectPtCloud[i].y;
		p.z = selectedObjectPtCloud[i].z;

		cloud.points.push_back(p);
	}	
	cloud.width = numberOfPoints;
	cloud.height = 1;
	*cloudPtr = cloud;

	/*
	 * The voxel downsampling parameters must be initialized
	 */
	const int NUM_SCALES = 4;
	const float MAX_VOXEL = 0.03f;
	const float MIN_VOXEL = 0.003f;
	const float SCALE_DIFF = (MAX_VOXEL - MIN_VOXEL) / (float) NUM_SCALES;
	float gridSizes[NUM_SCALES];
	for (int s = 0; s < NUM_SCALES; s++) {
		gridSizes[s] = MAX_VOXEL - (s * SCALE_DIFF);
	}

	/* 
	 * Calculate first error differences
	 */
	/// Variables pertaining to multi-scale voxelization algorithm
	double errorThreshold = 2.0;
	double errorDiff = 1000.0;
	double errorValue = 0;
	double prevErrorValue = 0;

	estimateInitialParameters(cloud, sqEstimate);
	errorValue = qualityOfFit(cloud, sqEstimate);
	errorDiff = abs(prevErrorValue - errorValue);
	prevErrorValue = errorValue;

	if (verbose) {
		std::cout << ">> Performing object pose estimation using Superquadrics\n";
	}

	/*
	 * Multi-scale voxelization for pose estimation
	 */
	for (int j = 0; j < NUM_SCALES && errorDiff >= errorThreshold; j++) {
		if (j != NUM_SCALES - 1) {
			pcl::VoxelGrid<pcl::PointXYZ> grid;
			grid.setInputCloud (cloudPtr);
			grid.setLeafSize (gridSizes[j], gridSizes[j], gridSizes[j]);
			grid.filter (downsampledCloud);

		} else {
			downsampledCloud = cloud;
		}

		/*
		 * Estimate the dimensions of cloud (in order to get a more accurate fit)
		 */
		estimateInitialParameters(downsampledCloud, sqEstimate);
		
		/*
		 * Initialize the superquadric parameters based on the cloud dimensions
		 */
		initParams.min.iterations = 30;		
		initParams.min.a1.type = BOUNDED;
		initParams.min.a1.lowerBound = 0.020f;
		initParams.min.a1.upperBound = sqEstimate.a1 + 0.1f;
		initParams.a1 = 0.05f;
	
		initParams.min.a2.type = BOUNDED;
		initParams.min.a2.lowerBound = 0.020f;
		initParams.min.a2.upperBound = sqEstimate.a2 + 0.1f;
		initParams.a2 = 0.05f;

		initParams.min.a3.type = BOUNDED;
		initParams.min.a3.lowerBound = 0.020f;
		initParams.min.a3.upperBound = sqEstimate.a3 + 0.1f;
		initParams.a3 = 0.05f;

		initParams.min.e1.type = BOUNDED;
		initParams.min.e1.lowerBound = 0.1f;
		initParams.min.e1.upperBound = 2.0f;
		initParams.e1 = 1.0f;

		initParams.min.e2.type = BOUNDED;
		initParams.min.e2.lowerBound = 0.1f;
		initParams.min.e2.upperBound = 2.0f;
		initParams.e2 = 1.0f;
	
		initParams.min.phi.type = UNLIMITED;
		initParams.min.theta.type = UNLIMITED;
		initParams.min.psi.type = UNLIMITED;
		initParams.min.phi.value = 1.0f;
		initParams.min.theta.value = 1.0f;
		initParams.min.psi.value = 1.0f;
	
		initParams.min.px.type = UNLIMITED;
		initParams.min.py.type = UNLIMITED;
		initParams.min.pz.type = UNLIMITED;
		
		if (ope::allowTapering) {
			initParams.min.kx.type = BOUNDED;
			initParams.min.kx.lowerBound = -1.0f;
			initParams.min.kx.upperBound = 1.0f;
			initParams.min.kx.value = 0.0f;
			initParams.kx = 1.0f;
		
			initParams.min.ky.type = BOUNDED;
			initParams.min.ky.lowerBound = -1.0f;
			initParams.min.ky.upperBound = 1.0f;
			initParams.min.ky.value = 0.0f;
			initParams.ky = 1.0f;

		} else {
			initParams.min.kx.type = UNCHANGED;
			initParams.min.kx.value = 0.0f;	
			initParams.min.ky.type = UNCHANGED;
			initParams.min.ky.value = 0.0f;

		}		

		performShapeFitting(downsampledCloud, initParams, bestParams);		
		errorValue = qualityOfFit(cloud, bestParams);
		errorDiff = abs(prevErrorValue - errorValue);
		prevErrorValue = errorValue;

		/*
		 * The parameters found at this level are used to initialize the successive one
		 */
		bestParams.copyTo(initParams);
		downsampledCloud.clear();
	}

	initParams.copyTo(bestParams);		
	
	return bestParams;
}


//this function will return the object closest to the seed point
SQParameters ObjectPoseEstimator::getClosestObject(pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointXYZ& seedPoint){
	 //Initialize everything
	//init(opeSettings);

	SQParameters sqParams;
	//PointCloudCapture cap;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
	bool detectedObjects;
	size_t desiredObjIdx = 0;

	/*
	 * Capture point cloud
	 */
	//cap.run(*cloudPtr);
	

	/*
	 * Generate object models based on extracted clusters
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(&cloud);
	ObjectModelGenerator<pcl::PointXYZRGB> generator(cloudPtr);
	detectedObjects = generator.generateObjectModels();


	/*
	 * Choose the object whose pose (position and orientation) you wish to calculate
	 */
	if (detectedObjects) {

		int numObjects = generator.getNumDetectedObjects();
		/* calculate the object that seed point belongs to */
		desiredObjIdx = Utils::getDesiredObject(cloudPtr, generator.getBoundingBoxes());
		
		/*
		 * Calculate the object pose using Superquadrics
		 */
		sqParams = ObjectPoseEstimator::calculateObjectPose(generator.objects[desiredObjIdx].objectCloud);
		cout << sqParams;

	} else {
		cout << ">> NO OBJECTS WERE DETECTED!" << endl;
	}

	return sqParams;
}


SQParameters ObjectPoseEstimator::run(const OPESettings& opeSettings) {
	// Initialize everything
	init(opeSettings);

	SQParameters sqParams;
	PointCloudCapture cap;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
	bool detectedObjects;
	size_t desiredObjIdx = 0;

	/*
	 * Capture point cloud
	 */
	cap.run(*cloudPtr);
	

	/*
	 * Generate object models based on extracted clusters
	 */
	ObjectModelGenerator<pcl::PointXYZRGB> generator(cloudPtr);
	detectedObjects = generator.generateObjectModels();


	/*
	 * Choose the object whose pose (position and orientation) you wish to calculate
	 */
	if (detectedObjects) {
		desiredObjIdx = Utils::getDesiredObject(cloudPtr, generator.getBoundingBoxes());
		
		/*
		 * Calculate the object pose using Superquadrics
		 */
		sqParams = ObjectPoseEstimator::calculateObjectPose(generator.objects[desiredObjIdx].objectCloud);
		cout << sqParams;

	} else {
		cout << ">> NO OBJECTS WERE DETECTED!" << endl;
	}

	return sqParams;
}


} // ope