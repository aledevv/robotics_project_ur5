//ROS
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "robotics_project_ur5/GetBrickPose.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");
    ros::NodeHandle node_handler;
    ros::Publisher pub = node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::ServiceClient service_client = node_handler.serviceClient<robotics_project_ur5::GetBrickPose>("GetBrickPose");
    robotics_project_ur5::GetBrickPose srv;

     if (service_client.call(srv))
     {  
         for (int i = 0; i < srv.response.numBricks; ++i)
        { 
            // ROS_INFO("pose: %ld, %ld, %ld, %ld", srv.response.pose[0].position.x, srv.response.pose[0].position.y, srv.response.pose[0].position.z, srv.response.pose[0].orientation.z);
            // ROS_INFO("length: %ld", srv.response.numBricks[0]);
            // ROS_INFO("label: %s", srv.response.label[0]);
            std::cout << "pose x brick " << i + 1 << ":\n" << srv.response.pose[i].position.x << std::endl;
            std::cout << "pose y brick " << i + 1 << ":\n" << srv.response.pose[i].position.y << std::endl;
            std::cout << "pose z brick " << i + 1 << ":\n" << srv.response.pose[i].position.z << std::endl;
            std::cout << "orientation z brick " << i + 1 << ":\n" << srv.response.pose[i].orientation.z << std::endl;
        }
        
        ROS_INFO("COMPLETED");
     }

    ros::spin();

}