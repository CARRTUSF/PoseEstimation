#include "ObjectSelector.h"


ObjectSelector::ObjectSelector()
{
}


ObjectSelector::~ObjectSelector()
{
}


void ObjectSelector::mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	
	if (event.getType() == pcl::visualization::MouseEvent::MouseDblClick) {
		std::cout << "Double clicked at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
		viewer->addText ("clicked here", event.getX(), event.getY());
	}
}