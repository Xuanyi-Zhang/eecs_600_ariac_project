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
#include<tf/message_filter.h>
#include <std_msgs/Float64.h>
#include<iostream>
#include<fstream>
#include <math.h> 

#define MODEL_DISTANCE_THRESHOLD	0.03

laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud cloud;
sensor_msgs::LaserScan g_LaserData; 

double g_maxZ=0.0;
double g_minZ=10.0;
double g_averageZ=0.0;

std::ofstream outfile;

double g_beginTime=0.0; 
double g_min_model_distance= 10.0;
std::string g_closest_model; 

//Alternatively we could not use a transform and just directly
//transform the laser scan data to x , y coordinates as shown below. 
//We'll keep this code here for reference 

//geometry_msgs::Point32 points;
//projector_.projectLaser(laserData, cloud);

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserData) 
{ 
	// just populate our global variable for the laser scan data when it comes in
	// We used to send the point cloud information out here. But this resulted in a 
	// race condition with the send_transform class. At times we could not 
	// update the ariac camera's transform with the timestamp from the /world
	// transform before attempting the point cloud processing, which resulted in
	// an exception 
	
	g_LaserData=*laserData; 
} 

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &imageData) 
{ 

	int modelsInView=imageData->models.size(); 	
	
	double model_distance=10.0;
	g_min_model_distance=10.0; // reset this to something large so we get a new
	// minimum value everytime we receive new logical camera information
	
	for(int i = 0; i < modelsInView; ++i)
	{	
		model_distance=fabs(imageData->models[i].pose.position.y); 
		if(model_distance < g_min_model_distance)
		{
			g_min_model_distance=model_distance; 
			g_closest_model=imageData->models[i].type;
			
			//only print out this information to the console when it's really interesting
			// when we know the part is close to the center of the scanner 
			std::cout << "Min model distance: " <<  g_min_model_distance<< '\n';
			std::cout << "Closest model: " <<  g_closest_model<< '\n';
			std::cout << "Average Z: " <<  g_averageZ<< '\n';
			std::cout << "Max Z height: " <<  g_maxZ<< '\n';
			std::cout << "Min Z height: " <<  g_minZ<< '\n';
		}
	}
	
} 

void calculateZStatistics(void) 
{ 
	// just some basic statistics about the z heights from our laser scanner
	// this illustrate that some of our parts are really hard to decipher from noise
	
	double totalZ=0;
	double currentZ=0;
	
	//reset statistics 
	g_maxZ=0;
	g_minZ=10.0;
	g_averageZ=0.0; 
	
	double counter = cloud.points.size(); 	
	for(int i=0;i<cloud.points.size(); i++)
	{
		currentZ=cloud.points[i].z; 
		totalZ=currentZ+totalZ; 
		
		if(currentZ > g_maxZ)
		{
			g_maxZ=currentZ; 
		}
		
		if(currentZ < g_minZ)
		{
			g_minZ=currentZ; 
		}
		
	}
	
	g_averageZ = totalZ / counter; 
	
} 

void writeToCSV(void) 
{ 
	//define some boolean fields to track if parts are near the scanner
	bool part1 = 0; 
	bool part2 = 0; 
	bool part3 = 0; 
	bool part4= 0; 
	bool pistonRod=0; 
	bool gear=0;
	bool gasket=0;
	bool pulley=0;
	
	
	
	double currentTime = ros::Time::now().toSec(); 
	currentTime=currentTime-g_beginTime;
	
	outfile.open("laser_scan_log.csv", std::ofstream::app);
    outfile<< currentTime <<"," << g_maxZ <<"," <<  g_minZ <<"," <<  g_averageZ <<",";
    for(int i=0; i<85; i++)
    {
		outfile << cloud.points[i].z << ",";
		
	}  
	
	outfile << g_closest_model << "," << g_min_model_distance << ",";
	if (g_min_model_distance < MODEL_DISTANCE_THRESHOLD)
	{
		if(g_closest_model == "part1")
		{
			part1 = 1;
		}
		
		else if (g_closest_model == "part2")
		{
			part2 = 1;
		}
		
		else if (g_closest_model == "part3")
		{
			part3 = 1;
		}
		
		else if (g_closest_model == "part4")
		{
			part4 = 1;
		}
		
		else if (g_closest_model == "piston_rod_part")
		{
			pistonRod = 1;
		}
		
		else if (g_closest_model == "gasket_part")
		{
			gear = 1;
		}
		
		else if (g_closest_model == "gear_part")
		{
			gasket = 1;
		}
		
		else if (g_closest_model == "pulley_part")
		{
			pulley = 1;
		}
		
	}
	
	outfile << part1 << "," << part2 << "," << part3 << "," << part4 << "," << pistonRod << "," << gear << "," << gasket << "," << pulley <<std::endl; 
	
	outfile.close();
} 

int main(int argc, char **argv) 
{ 
	ros::init(argc,argv,"log_scan_data"); 
	ros::NodeHandle n; 

	tf::TransformListener listener_;	
	tf::StampedTransform transform;
  
	ros::Subscriber my_subscriber_object= n.subscribe("/ariac/laser_profiler_1",1,laserScanCallback); 
	ros::Subscriber my_subscriber_object1= n.subscribe("/ariac/logical_camera_1",1,logicalCameraCallback); 
	
	ros::Publisher my_publisher_object = n.advertise<sensor_msgs::PointCloud>("convertToPointCloud", 1);
	ros::Rate naptime(100); //create a ros object from the ros “Rate” class; 
	
	g_beginTime= ros::Time::now().toSec();
	
	// write the headers for our logfile. Currently any file in the existing directory will be 
	// overwritten everytime this program is run. Buyer beware!!
	
    outfile.open("laser_scan_log.csv");
    outfile<< "T_s,Z_max,Z_min,Z_avg,";
    for(int i=0; i<85; i++)
    {
		outfile << i << ",";
		
	}  
	
	outfile<<"Part_String,Y_Distance,Part1,Part2,Part3,Part4,Piston_Rod,Gear_Part,Gasket,Pulley"<< std::endl;
	outfile.close();
    
	while (ros::ok()) 
		{
			try
			{
				// give it a second to find the transform. This shouldn't take nearly that long
				// you need to be running the accompanying program, send_transform in order to see
				// this transformation however. 
				
				// a couple errors get generated shortly after this file is run. I'm not too worried about this
				// but could look into it further.  
				
				listener_.waitForTransform("/world", "ariac/laser_profiler_1_frame", 
				ros::Time(0), ros::Duration(1.0));
				
				listener_.lookupTransform("/world", "ariac/laser_profiler_1_frame", 
				ros::Time(0), transform);
				projector_.transformLaserScanToPointCloud("/world",  g_LaserData, cloud, listener_);
				
				// once the transform is done publish our transformed data. If you take a look at RVIZ, looks like
				// our cloud looks the same as theirs does after they convert from the laserscan type. I'd say 
				// we're doin something right!
				my_publisher_object.publish(cloud);
				
				std::cout << "Size of pointCloud points container: " <<  cloud.points.size() << '\n';
				
				calculateZStatistics(); 
				
				// record a logfile. We could put this in a switch statement to enable or disable this, 
				// but I'm not too woried about this now. 
				writeToCSV();
				
			}
			
			catch (tf::TransformException &ex) 
			{
				ROS_ERROR("%s",ex.what());
			}
			
			ros::spinOnce(); 
			naptime.sleep(); 
	}
  return 0; // should never get here, unless roscore dies 
} 
