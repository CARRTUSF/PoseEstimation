#include "PointCloudCapture.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include "Common.h"
#include "Utils.h"

using namespace ope;

PointCloudCapture::PointCloudCapture() : ptCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), 
	filteredPtCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>) { 

}


PointCloudCapture::~PointCloudCapture() {
	
}


void PointCloudCapture::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	ptCloudPtr = cloud;
}


void PointCloudCapture::run(pcl::PointCloud<pcl::PointXYZRGB>& ptCloud) {
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
		boost::bind (&PointCloudCapture::cloudCallback, this, _1);

	if (verbose) {
		Utils::printCurrentDateTime();
		std::cout << ">> Capturing point cloud using the Kinect" << std::endl;
	}

	interface->registerCallback (f);
	interface->start ();

	// Pause for 1 second to ensure that a point cloud is captured
	boost::this_thread::sleep (boost::posix_time::seconds (1));
	
	interface->stop ();

	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (ptCloudPtr);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (ope::minimumTgtDepth, ope::maximumTgtDepth);
	pass.filter (*filteredPtCloudPtr);

	Utils::convertPointCloud(*filteredPtCloudPtr, ptCloud);

	if (verbose) {
		std::cout << ">>\t ...done" << std::endl;
	}
				
	// Only for debugging purposes
	if (ope::doDebug) {
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Debugging Viewer"));
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(filteredPtCloudPtr);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGBA> (filteredPtCloudPtr, rgb, "Filtered Cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Filtered Cloud");
		viewer->initCameraParameters();
		viewer->resetCameraViewpoint("Filtered Cloud");

		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds(10));
		}	
			
		//pcl::io::savePLYFile("CapturedTestCloud.ply", *filteredPtCloudPtr);
	}

}
