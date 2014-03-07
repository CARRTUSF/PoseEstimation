/*
 * Software License Agreement (BSD License)
 *
 *  Object Pose Estimation (OPE) - www.cse.usf.edu/kkduncan/ope
 *  Copyright (c) 2013, Kester Duncan
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *	\file	ObjectPoseEstimator.h
 *	\brief	Performs object pose estimation using superquadrics
 *	\author	Kester Duncan
 */
#pragma once
#ifndef __OBJECT_POSE_ESTIMATOR_H__
#define __OBJECT_POSE_ESTIMATOR_H__
//#include <pcl/point_types.h>
//#include <Eigen/StdVector>

/// Forward declarations to avoid header file includes
class SQParameters;
namespace pcl {

struct PointXYZRGB;
struct PointXYZ;
template <class PointXYZRGB> class PointCloud;

} // pcl


/// Namespace \a ope that contains all the functions and types relevant for estimating an object's pose
namespace ope {


/**
 * \brief User-customizable program settings
 */
class OPESettings {
public:
	/// Minimum depth value for target scene area. Used by the <code>PassThrough</code> filter
	float minTgtDepth;

	/// Maximum depth value for target scene area
	float maxTgtDepth;

	/// Minimum height of object hypotheses in the scene
	float minObjHeight;

	/// Maximum height of object hypotheses in the scene
	float maxObjHeight;

	/// Determines whether status updates are output
	bool verbose;

	/// Determines the amount of error minimization iterations for superquadric fitting
	int minIterations;

	/// Determines whether superquadric shape tapering is allowed when estimating pose
	bool allowTapering;

	/// Object voxel size
	float objVoxelSize;

	/// Show a debugging viewer when point cloud is captured
	bool allowDebug;

public:
	/**
	 * \brief Constructor
	 * \post Initializes all member variables to their default values
	 */
	OPESettings() : minTgtDepth(0.2f), maxTgtDepth(1.8f), minObjHeight(0.03f), maxObjHeight(0.30f), 
		verbose(true), allowTapering(false), objVoxelSize(0.003f), allowDebug(false) {

	}

};


/**
 * \brief Performs object pose estimation using the parametric superquadric shape model
 * \details Superquadrics are a family of parametric shapes that include superellipsoids, 
 * supertoroids, and superhyperboloids with one and two parts. They are appealing for 
 * robotic applications by nature of their definition. We focus on the superellipsoid which 
 * is useful for a volumetric part-based description. Given the parameters that define a 
 * superquadric, the shape and pose information can be easily extracted as well as volumes 
 * and moments of inertia. They are compact in shape and have a closed surface. Moreover, 
 * superquadrics exhibit tri-axis symmetry, which is a characteristic well approximated by 
 * many household objects.
 *
 * Superquadrics can be defined in an object centered coordinate system with five variables 
 * and in a general coordinate system by eleven independent variables. The variables 
 * \f$ a_1, a_2, a_3 \f$ are the scaling dimensions along the x, y, and z 
 * axes of the superquadric, \f$ e1, e2 \f$ are the factors which determine the 
 * superquadric's shape ranging from from 0.1 to 1, and 
 * \f$ (n_x, n_y, n_z, o_x, o_y, o_z, a_x, a_y, a_z, p_x, p_y, p_z) \f$ are the twelve parameters 
 * of the homogeneous transformation matrix that is a result of a rotation and translation 
 * of the world coordinate frame. 
 */
class ObjectPoseEstimator
{
public:
	/**
	 * \brief Calculate the pose of the object represented by the point cloud provided.
	 * \details The pose is estimated by using the algorithm outlined in the ICRA 2013 paper
	 * \b "Multi-scale Superquadric Fitting for Efficient Shape and Pose Recovery of Unknown Objects"
	 * by Kester Duncan
	 * http://www.cse.usf.edu/~kkduncan
	 * 
	 */
	static __declspec(dllexport) SQParameters calculateObjectPose(pcl::PointCloud<pcl::PointXYZRGB>& selectedObjectPtCloud);

	/**
	 * \brief Run the pose estimation algorithm
	 * \details This function executes as follows: Capture a point cloud, extract the table objects 
	 * from the point cloud, present a viewing window to the user in order for them to select the
	 * target object point cloud using its index, perform pose estimation on the object point cloud,
	 * and finally return the results in a \a SQParameters object.	 *
	 * \return An \a SQParameters object containing the 13 superquadric parameters.
	 */
	static __declspec(dllexport) SQParameters run(const OPESettings& opeSettings);

	static __declspec(dllexport) SQParameters getClosestObject(pcl::PointCloud<pcl::PointXYZRGB>&cloudPtr , pcl::PointXYZ& seedPoint);

private:
	/**
	 * \brief Initializes program-wide OPE properties
	 */
	static void init(const OPESettings& opeSettings);

};


} // ope

#endif /* __OBJECT_POSE_ESTIMATOR_H__ */
