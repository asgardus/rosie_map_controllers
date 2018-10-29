#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <math.h>
#include <iostream>
#include <pcl_ros/transforms.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

float pi = 3.14159265359;
float resolution;
int width = 1;
int height = 1;
std_msgs::Int8 *occGrid;

ros::Time load_time;

void initializeMap(){
	width = 300;
	height = 300;
	resolution = 0.01f;
	occGrid = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*width*height);
	for(int i = 0; i < height; ++i){
		for(int j = 0; j < width; ++j){
			if(i == 0){
				occGrid[i*height+j].data = 150;
			}else if(i == height-1){
				occGrid[i*height+j].data = 150;				
			}else if(j == 0){
				occGrid[i*height+j].data = 150;
			}else if(j == width-1){
				occGrid[i*height+j].data = 150;			
			}else if(i == 100 && j < 200){
				occGrid[i*height+j].data = 150;
			}else{
				occGrid[i*height+j].data = 0;
			}
		}
	}
}

void mapUpdateCallback(){
	
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_map_provider");

    ros::NodeHandle n;
 
    ros::Publisher grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/rosie_occupancy_grid",1);

    ros::Rate loop_rate(5);
 
	tf::Transform transform;
    static tf::TransformBroadcaster br;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion qtf;
    qtf.setRPY(0, 0, 0);
    transform.setRotation( qtf );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

	initializeMap();

	load_time = ros::Time::now();
	int seq = 0;
    while(ros::ok()){
    	nav_msgs::OccupancyGrid mapgrid;

		mapgrid.header.seq = seq++;
		mapgrid.header.stamp = ros::Time::now();
		mapgrid.header.frame_id = "map";

		mapgrid.info.origin.position.x = 0;
		mapgrid.info.origin.position.y = 0;
		mapgrid.info.origin.position.z = 0;
		mapgrid.info.origin.orientation.x = 0;
		mapgrid.info.origin.orientation.y = 0;
		mapgrid.info.origin.orientation.z = 0;
		mapgrid.info.origin.orientation.w = 1;

		mapgrid.info.map_load_time = load_time;
		mapgrid.info.resolution = resolution;
		mapgrid.info.width = width;
		mapgrid.info.height = height;

		mapgrid.data.clear();

		//mapgrid.data = (std_msgs::Int8*)malloc(sizeof(std_msgs::Int8)*width*height);
		for(int i = 0; i < width*height; ++i){
			mapgrid.data.push_back(occGrid[i].data);
		}

	    grid_publisher.publish(mapgrid);			
	    ros::spinOnce();
    	loop_rate.sleep();
    }
}
