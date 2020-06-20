#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
//include ROS msg type headers and libraries

//PID Control Parameters
double kp = 5;
double ki = 0.003;
double kd = 0.0000001;
//initial value
double error = 0.0;
double integral = 0.0;
double prev_error = 0.0;
bool Safety_flag = false;
double steer_angle = 0.0;
double velocity = 0.0;

class Wall {
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
    Wall() {
        n = ros::NodeHandle();
        speed = 0.0;
        //create ROS subscribers and publishers
        pub_drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 100);
	pub_brake = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);
        pub_bool = n.advertise<std_msgs::Bool>("/brake_bool", 1);
	scan = n.subscribe("/scan",100, &Wall::scan_callback,this);
	odom = n.subscribe("/odom",100, &Wall::odom_callback,this);
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        //update current speed
	geometry_msgs::TwistWithCovariance temp = odom_msg->twist;
	geometry_msgs::Twist temp2 = temp.twist;
        speed = temp2.linear.x;
	ROS_INFO("The speed is :[%f]",speed);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
	double min_angle = scan_msg->angle_min;
	double max_angle = scan_msg->angle_max;
	double increment_angle = scan_msg->angle_increment;
	int scan_times = int((max_angle-min_angle)/increment_angle);
	std::vector<float> dis = scan_msg->ranges;

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
	steer_angle = Wall::Follow_Wall_Left(dis);
	velocity = Set_velocity(steer_angle);

	ackermann_msgs::AckermannDriveStamped drv;
	//drv.header.stamp = ros::Time.now();
	drv.header.frame_id = "laser";
	drv.drive.steering_angle = steer_angle;
	    
	//drv.drive.steering_angle = M_PI/18;
	drv.drive.speed = velocity;
	pub_drive.publish(drv);
	//usleep(1000);
    }

    double Follow_Wall_Right(std::vector<float> dis){
	double a = dis.at(405), b = dis.at(270);
	double theta = M_PI/4;
	double alpha = atan((a*cos(theta)-b)/a*sin(theta));
	double Dt0 = b*cos(alpha);
	double L = 0.1;
	double Dt1 = Dt0 + L*sin(alpha);
	prev_error = error;
	error = 1 - Dt1;
	integral += error;
	steer_angle =  kp*error + ki*integral + kd*(error - prev_error)/0.01;
	return steer_angle;	
    }

    double Follow_Wall_Left(std::vector<float> dis){
	double a = dis.at(674), b = dis.at(809);
	double theta = M_PI/4;
	double alpha = atan((a*cos(theta)-b)/a*sin(theta));
	double Dt0 = b*cos(alpha);
	double L = 0.1;
	double Dt1 = Dt0 + L*sin(alpha);
	prev_error = error;
	error = Dt1 -1;
	integral += error;
	steer_angle =  kp*error + ki*integral + kd*(error - prev_error)/0.01;
	return steer_angle;	
    }

    double Set_velocity(double steer_angle){
	double velocity;
	double steer_angle_pi = (steer_angle/M_PI)*180;
	if(steer_angle_pi >= 0 && steer_angle_pi <= 10){
		velocity = 1.5;
	}
	else if(steer_angle_pi > 10 && steer_angle_pi <= 20){
		velocity = 1.0;
	}
	else{
		velocity = 0.5;
	}
	return velocity;
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
    ros::init(argc, argv, "wall_follow");
    Wall wf;
    ros::spin();
    return 0;
}
