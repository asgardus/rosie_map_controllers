#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_mapping");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);
 
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    tf::Quaternion q;

    while(ros::ok()){

	broadcaster.sendTransform(
	    tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.125, 0.0, 0.19)), // vector3 = position laser
	        ros::Time::now(), "my_frame", "laser_frame"));        
        loop_rate.sleep();

    }
}
