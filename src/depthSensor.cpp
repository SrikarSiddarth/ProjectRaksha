#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
// #include <tf2_ros/transform_broadcaster.h>

// tf2_ros::TransformBroadcaster tf_broadcaster_;


int u = 0, v = 0;
geometry_msgs::Point p;

void xy_cb(const geometry_msgs::Point l){
	u = l.x;
	v = l.y;
}

/**
Function to convert 2D pixel point to 3D point by extracting point
from PointCloud2 corresponding to input pixel coordinate. This function
can be used to get the X,Y,Z coordinates of a feature using an 
RGBD camera, e.g., Kinect.
*/
void depth_cb(const sensor_msgs::PointCloud2 pCloud)
{
	
	
  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

 // put data into the point p
  p.x = X;
  p.y = Y;
  p.z = Z;
  std::cout<<"X : "<<X<<" Y : "<<Y<<" Z : "<<Z<<std::endl;
  
}

int main(int argc, char **argv){
	ros::init(argc, argv,"depthSensor");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("/depthSensor/points",1000);
	
	ros::Subscriber sub = n.subscribe("/camera/depth/points",1000,depth_cb);
	ros::Subscriber sub1 = n.subscribe("/xyPoint",1000,xy_cb);

	ros::Rate loop_rate(10);

	while(ros::ok){
		pub.publish(p);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}