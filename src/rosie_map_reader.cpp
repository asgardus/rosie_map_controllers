#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
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
visualization_msgs::Marker **markers;
std::vector<float> **extMarkersX;
std::vector<float> **extMarkersY;
int **occGrid;
/*mav_msgs::Odometry offset;
offset.pose.pose.orientation.x = 0;
offset.pose.pose.orientation.y = 0;
offset.pose.pose.orientation.z = 0;
offset.pose.pose.orientation.w = 1;
*/
tf::Quaternion q;
tf::Transform transform;



void poseCallback(const nav_msgs::Odometry& odom){

    transform.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z) );
    
    q[0] = odom.pose.pose.orientation.x;
	q[1] = odom.pose.pose.orientation.y;
	q[2] = odom.pose.pose.orientation.z;
	q[3] = odom.pose.pose.orientation.w;
    transform.setRotation( q );

    tf::Transform tfmap;
    static tf::TransformBroadcaster br;
    transform.setOrigin( tf::Vector3(0, 0, 0) );
    tf::Quaternion qmap;
    qmap.setRPY(0, 0, 0);
    transform.setRotation( qmap );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/map"));
}

void mapCallback(const visualization_msgs::MarkerArray::ConstPtr& msg){
  
    markers = msg->markers;

}

void update(){

}

int main(int argc, char **argv){
    ros::init(argc, argv, "map_subscriber");

    ros::NodeHandle n;
 
    ros::Subscriber map_sub = n.subscribe<visualization_msgs::MarkerArray>("/maze_map", 10, mapCallback); 
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, poseCallback); 
    ros::Publisher point_cloud_publisher = n.advertise<sensor_msgs::PointCloud>("/map_cloud",100);
    ros::Publisher grid_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map_grid",100);

    ros::Rate loop_rate(10);
 
    tf::TransformListener listener;
    //generate point cloud map
    int size = (sizeof(msg.markers)/sizeof(*msg.markers));
    **markers = (Markers[] **)malloc(size * sizeof(int *));
    int sum_dist = 0;
    float step = 0.01;

    for (int i=0; i<= size ; i=i+4 ){
        //calc a function on start & Endpoint
        float x1 = markers[i];
        float y1 = markers[i+1];
        float x2 = markers[i+2];
        float y2 = markers[i+3];
        sum_dist += (int) sqrt((y2-y1)^2+(x2-x1)^2)/step;
    }
 
    **extMarkersX = (std::vector<float> **)malloc(((int) sum_dist)*sizeof(int*));
    **extMarkersY = (std::vector<float> **)malloc(((int) sum_dist)*sizeof(int*));
    int count = 0;

    for (int i=0; i<= size; i=i+4){
        float x1 = markers[i];
        float y1 = markers[i+1];
        float x2 = markers[i+2];
        float y2 = markers[i+3];
        float step = 0.01;
        float dist = sqrt((y2-y1)^2+(x2-x1)^2);
        float slope = (x2-x1)/dist;
    
        for (float d = 0; d < dist; d = d+step){
            extMarkersX[count] = d*cos(slope) +x1
            extMarkersY[count] = d*sin(slope)+y1;
            count++;
        }
    }

    //calc occupancy grid size
    float maxX = markers[3];
    float maxY = markers[8];
    //generate empty occupancy grid
    occGrid = new int*[maxX*100+1];
    for(int j = 0; j<=(maxX*100);j++){
        occGrid[j]= new int[maxY+1]
    }

    extMarkersX = extMarkersX*100;
    extMarkersY = extMarkersY*100;a
    for(int j = 0; j<= size; j++){
        for(int k = 0; k<=maxY; k++){
            if(std::find(std::begin(extMarkersX),std::end(extMarkersX),j) && std::find(std::begin(extMarkersY),std::end(extMarkersY),k)){
               occGrid[j][k] = 1 ;
            } else {
                occGrid[j][k] = 0;
            }
            
        }
    }

    int8[] **occGrid_int = new int8*[4+(maxX*100+1)*(maxY*100+1)];
    occGrid_int[0]= 256;
    occGrid_int[1]= maxX-256;
    occGrid_int[2]= 256;
    occGrid_int[3]= maxY-256;
    count = 4;
    for(int i = 0; i<= (maxX*100); i=i+2){
        for(int j = 0; i<= (maxX*100); i=i+2){
            occGrid_int[count] = occGrid[i][j];
            count++;
        }
    }

    while(ros::ok()){
        sensor_msgs::PointCloud mapcloud;
        mapcloud.header.stamp = ros::Time::now();
        mapcloud.header.frame_id = "map_frame";
        mapcloud.points.resize(360);

        mapcloud.channels.resize(1);
        mapcloud.channels[0].name = "intensities";
        mapcloud.channels[0].values.resize(360);
        for(unsigned int i = 0; i<360; i++){
        mapcloud.points[i].z = 0; 
            mapcloud.points[i].x = extMarkersX[i];
            mapcloud.points[i].y = extMarkersY[i];
            mapcloud.channels[0].values[i] = intensities[i];
        }
    point_cloud_publisher.publish(mapcloud); 
    grid_publisher.publish(mapgrid);

    ros::spinOnce();
    loop_rate.sleep();

    }
}
