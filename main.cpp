#include "myKinect.h"

using namespace std;

PointCloudT::Ptr cloud_temp(new PointCloudT);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing) {

	if (event.getKeySym() == "space" && event.keyDown()) {
		cout << "save point cloud......." << endl;
		pcl::io::savePCDFileBinary("./data/myKinect.pcd", *cloud_temp);
		cout << "save done." << endl;
	}
}

int main()
{

	CKinectWithLibfreenect2 myKinect("0");

	if (!myKinect.InitKinectSensor())
	{
		cout << "Initialization failed! Exiting..." << endl;
		return -1;
	}

	myKinect.OpenKinectDevice();

	myKinect.deviceName = "myKinect";


	myKinect.WriteCameraParams("./data/camera_ir_" + myKinect.deviceName + ".yaml");

	int frameNum = 0;


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->addCoordinateSystem(1.0);


	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)NULL);

	while (!viewer->wasStopped())
	{
		if (myKinect.update()) {
			cloud_temp->clear();
			myKinect.getPointCloud(cloud_temp);
			if (!viewer->updatePointCloud(cloud_temp, "cloud")) {
				viewer->addPointCloud(cloud_temp, "cloud");
			}
		}

		viewer->spinOnce();
	}
	return 0;
}

