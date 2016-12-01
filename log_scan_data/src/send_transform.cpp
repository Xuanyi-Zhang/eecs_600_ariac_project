// Written for EECS 600: Special Topics in ROS Programming
// Case Western Reserve University
// Mike Gallagher
// mjg152@case.edu



#include<ros/ros.h> 
#include<osrf_gear/LogicalCameraImage.h> 
#include<sensor_msgs/LaserScan.h> 
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Point32.h>
#include<laser_geometry/laser_geometry.h>
#include<tf/transform_listener.h>
#include<tf/tfMessage.h>
#include<cstdio>
#include<tf/transform_broadcaster.h>

laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud cloud;

tf::StampedTransform transform_;

void tfBroadcaster(const tf::tfMessage tfMessage) 
{ 
	//this was a major pain! tried many variations to get the transformLaserScanToPointCloud in 
	// logscan data to be happy! Gist is if we don't replace the time stamp in the transform we create
	// from the "world" to "laser_profiler_1_frame" frame we get an error about having to extrapolate 
	// backwards and forwards in time. For reference, we were getting timestamps within a millisecond of one
	// another and it was still causing exceptions. 
	
	//I also investigated using tf_message_filters for this purpose. This was a long road that I'm fairly certain
	//would have solved the problem. In fact if you look at the RVIZ source for how they handle this problem 
	//I'm fairly certain that's how they resolved it https://github.com/ros-visualization/rviz/blob/kinetic-devel/src/rviz/default_plugin/laser_scan_display.cpp
	//That being said once I realized I could break the loop by slowing down when I transformed the laserscan data by putting this in a timed loop 
	// as opposed to in a call back function things began to work and I moved on...
	
	//This really should be fixed by the ARIAC simulator. Would be as simple as renaming the laser_profiler_1_frame to 
	//ariac/laser_profiler_1_frame. Would have saved us a ton of time...but here we are.  
	
	int currentROSTime = 0; 

	static tf::TransformBroadcaster broadcaster;
	static tf::TransformListener listener_;	

	ros::Time current_transform = ros::Time::now();

	currentROSTime=listener_.getLatestCommonTime("world", "laser_profiler_1_frame", current_transform, NULL);

	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(0,0,0)), current_transform ,"laser_profiler_1_frame", "ariac/laser_profiler_1_frame");
	broadcaster.sendTransform(transform_);
}  

int main(int argc, char **argv) 
{ 
	ros::init(argc,argv,"send_transform"); 
	ros::NodeHandle n;

	ros::Subscriber my_subscriber_object1= n.subscribe("/tf",1,tfBroadcaster); 

	while (ros::ok()) 
	{
		ros::spin(); 
	}
return 0; // should never get here, unless roscore dies 
} 
