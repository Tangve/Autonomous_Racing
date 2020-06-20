#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
//include ROS msg type headers and libraries

//initial value
bool Safety_flag = false;
double steer_angle = 0.0;
double velocity = 1.5;
double min_dis = 0.0;

class Reactive {
// The class that hadles wall following
private:
    ros::NodeHandle n;
    double speed;
    //create ROS subscribers and publishers
    ros::Subscriber scan;
    ros::Subscriber odom;
    ros::Publisher pub_drive;
    ros::Publisher pub_bool;
    ros::Publisher pub_brake;
    
public:
    Reactive() {
        n = ros::NodeHandle();
        speed = 0.0;
        //create ROS subscribers and publishers
        pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
	pub_brake = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);
        pub_bool = n.advertise<std_msgs::Bool>("/brake_bool", 1);
	scan = n.subscribe("/scan",100, &Reactive::scan_callback,this);
	odom = n.subscribe("/odom",100, &Reactive::odom_callback,this);
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        //update current speed
	geometry_msgs::TwistWithCovariance temp = odom_msg->twist;
	geometry_msgs::Twist temp2 = temp.twist;
        speed = temp2.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
	double min_angle = scan_msg->angle_min;
	double max_angle = scan_msg->angle_max;
	double increment_angle = scan_msg->angle_increment;
	int scan_times = int((max_angle-min_angle)/increment_angle);
	std::vector<float> dis = scan_msg->ranges;
	double min_obs = 200.0;

	for(int i=449 ; i<629 ; i++){
		if(dis.at(i)<min_obs){
			min_obs = dis.at(i);
		}
	}
	ROS_INFO("min_obs is :[%f]",min_obs);

	Safety(dis, min_angle, max_angle, increment_angle, scan_times);

        //publish drive/brake message if necessary
	ackermann_msgs::AckermannDriveStamped brk;
	std_msgs::Bool bol_true;
	std_msgs::Bool bol_false;
	brk.drive.speed = 0.0;
	bol_true.data = true;
	bol_false.data = false;
	if(Safety_flag){
	    pub_brake.publish(brk);
	    pub_bool.publish(bol_true);
 	    ROS_INFO("Stop!");
	}
	else{
	    pub_bool.publish(bol_false);	
	}  
	steer_angle = Reactive::Get_angle(dis,scan_times,min_angle,max_angle,increment_angle);
	ROS_INFO("The min is :[%f]",min_dis);
	ROS_INFO("The steer angle is:[%f]",steer_angle);
	
	ackermann_msgs::AckermannDriveStamped drv;
	drv.header.frame_id = "laser";
	drv.drive.steering_angle = (1-min_obs*0.7)*steer_angle;
	drv.drive.speed = velocity;
	pub_drive.publish(drv);
	//usleep(1000);
    }

    double Get_angle(std::vector<float> dis, int scan_time, double min_angle, double max_angle, double increment_angle){
	min_dis = 150.0;
	int min_idx = 0;
	double bubble = 0.1;
	double theta = 0.0;
	double min_theta_idx = 0.0;
	double max_theta_idx = 0.0;
	double furthest_dis = 0.0;
	double furthest_angle = 0.0;
	int furthest_idx = 0;
	int start_pt = 0;
	int end_pt = 0;
	//find min point
	for(int i=0 ; i<scan_time ; i++){
		if(dis.at(i)<min_dis){
			min_dis = dis.at(i);
			min_idx = i;
		}
	}
	//find bubble range
	theta = asin(bubble/min_dis);
	theta = 0.20;
	min_theta_idx = int((min_idx*increment_angle-theta)/increment_angle);
	max_theta_idx = int((min_idx*increment_angle+theta)/increment_angle);
	//find furtheset point
	if(scan_time-max_theta_idx>min_theta_idx){
		start_pt = int(max_theta_idx);
		end_pt = int(scan_time);
	}
	else{
		start_pt = int(0);
		end_pt= int(min_theta_idx);
	}
	for(int i=start_pt ; i<end_pt ; i++){
		if(dis.at(i)<=3 && dis.at(i)>furthest_dis){
			furthest_dis = dis.at(i);
			furthest_idx = i;
		}
	}
	//furthest_idx = int((end_pt-start_pt)/2);
	furthest_angle = (min_angle+furthest_idx*increment_angle)-(min_angle+539*increment_angle);
	return furthest_angle;
    }
    void Safety(std::vector<float> dis, double min_angle, double max_angle, double increment_angle, int scan_times){
	double speed_vector[scan_times];
	for(int i=0 ; i < scan_times ; i++){
		speed_vector[i] = cos(min_angle+i*increment_angle)*speed;
	}

	double ttc_vector[scan_times];
	for(int i=0 ; i < scan_times ; i++){
		if(speed_vector[i] < 0.00001){
			speed_vector[i] = 0.0;
		}
		ttc_vector[i] = dis.at(i)/speed_vector[i];
	}
	
	float min_ttc = 1.0f / 0.0f;
	for(int i=0 ; i < scan_times ; i++){
		if(ttc_vector[i] < min_ttc){
			min_ttc = ttc_vector[i];
		}
	}
	if(min_ttc < 0.3){
		Safety_flag = true;
	}
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "Reactive_method");
    Reactive RM;
    ros::spin();
    return 0;
}
