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
 *	\file	Common.h
 *	\brief	Defines common constants, types etc. used throughout OPE
 *	\author	Kester Duncan
 */
#pragma once
#ifndef __OPE_COMMON_H__
#define __OPE_COMMON_H__


/** \namespace ope
 *	Namespace where all the Object Pose Estimation functionality resides
 */
namespace ope {

	/// Minimum depth value for target scene area. Used by the <code>PassThrough</code> filter
	static float minimumTgtDepth = 0.2f;
	
	/// Maximum depth value for target scene area
	static float maximumTgtDepth = 3.0f;

	/// Minimum height of object hypotheses in the scene
	static float minimumObjHeight = 0.01f;

	/// Maximum height of object hypotheses in the scene
	static float maximumObjHeight = 0.50f;

	/// Determines whether status updates are output
	static bool verbose = true;

	/// Determines the amount of error minimization iterations for superquadric fitting
	static int minimumIterations = 30;

	/// Size of object voxels used by the Table Object Detector
	static float objectVoxelSize = 0.001f;

	/// Determines whether superquadric shape tapering is allowed when estimating pose
	static bool allowTapering = false;

	/// Allow debugging viewer
	static bool doDebug = false;

} /* ope */

#endif /* __OPE_COMMON_H__ */