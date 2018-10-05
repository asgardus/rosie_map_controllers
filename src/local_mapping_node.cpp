#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

double offsetIn = 0; // In direction of driving
double offsetCr = 0; // Cross direction of driving

std::vector<float> ranges(360,0);
float angle_increment;
float time_increment;
float scan_time;
// ros::Time frame_time;

void lidarCallback(const sensor_msgs::LaserScan& msg){

    ranges = msg.ranges;
    angle_increment = msg.angle_increment;
    time_increment = msg.time_increment;
    scan_time = msg.scan_time;
	
	
  std::copy(ranges.begin(), ranges.end(), std::ostream_iterator<float>(std::cout, " "));
  std::cout << std::endl;
    //ROS_INFO("Ranges: %f", ranges);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_mapping");

    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("/scan", 10, lidarCallback); 

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();

    }
}
