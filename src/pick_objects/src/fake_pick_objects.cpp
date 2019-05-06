#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <pick_msgs/PickedItem.h>
#include <tf/transform_datatypes.h>

#include <iostream>

namespace utils {
    double getRandDouble() {
        return (double)(rand() % 100) / 100.0;
    }
}

class PickObjects {
public:
    PickObjects(ros::NodeHandle nh, ros::NodeHandle private_nh) 
        : nh_(nh), private_nh_(private_nh)
    {
        private_nh_.param<double>("awaiting_interval_time", awaiting_interval_time_, 5.0);
        
        std::string picking_info_topic;
        private_nh_.param<std::string>("picking_info_topic", picking_info_topic, "picking_info");
        
        picking_pub_= nh_.advertise<pick_msgs::PickedItem>(picking_info_topic, 1);
    }
    
    pick_msgs::PickedItem getPickingItem(std::string frame_id, std::string action) {
        pick_msgs::PickedItem msg;
        msg.robot_status = action;
        double yaw = utils::getRandDouble() * M_PI;
        msg.pose.header.frame_id = frame_id;
        msg.pose.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = utils::getRandDouble() * 2.0 - 1.0;
        msg.pose.pose.position.y = utils::getRandDouble() * 2.0 - 1.0;
        msg.pose.pose.position.z = 0;
        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        return msg;
    }
    
    void run() {
        while (ros::ok()) {
            ROS_INFO("Awaiting for PICKING");
            ros::Duration(awaiting_interval_time_).sleep();
            pick_msgs::PickedItem msg1 = this->getPickingItem("map", "picking");
            picking_pub_.publish(msg1);
            
            ROS_INFO("Moving to DROPPING zone");
            ros::Duration(awaiting_interval_time_).sleep();
            pick_msgs::PickedItem msg2 = this->getPickingItem("map", "dropping");
            picking_pub_.publish(msg2);
            
            ros::spinOnce();
        }
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher picking_pub_;
    
    double awaiting_interval_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle nh, private_nh("~");
    
    PickObjects picker(nh, private_nh);
    picker.run();
    
    return 0;
}
