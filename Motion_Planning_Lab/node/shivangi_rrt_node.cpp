// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the node definition for RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf


#include "rrt/rrt.h"
#include <string>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <sstream>

const char* map_file_name = "./levine-blocked.csv";
const char* wp_file_name = "./waypts-2.csv";

mat read_csv(const char* file_name){
    std::string line;
    // char* path = std::filesystem::current_path();
    std::string file_path(file_name);
    mat m;

    ifstream file;

    file.open(file_path);

    if (file.is_open())
    {
        while(getline(file, line)){
            vec row;
            istringstream iss(line);
            string value;
            while (getline(iss, value, ',')){
                row.push_back(atof(value.c_str()));
            }
            m.push_back(row);
        }
        file.close();
    }
    else 
    {
        ROS_INFO("Rosrun only from team1_rrt/node directory");
        ROS_INFO("Incorrect name or unable to open file");
    }

    return m;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt");
    mat map_val = read_csv(map_file_name);
    mat waypoints = read_csv(wp_file_name);
    ros::NodeHandle nh;
    RRT rrt(nh, waypoints, map_val);
    ros::spin();
    return 0;
}
