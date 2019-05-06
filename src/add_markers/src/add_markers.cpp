#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <pick_msgs/PickedItem.h>

#include <iostream>

const double marker_lifetime = 2.0;

namespace utils {
    
    double getYawFromQuaternion(geometry_msgs::Quaternion orientation) {
        tf::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
        tf::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    visualization_msgs::Marker getTextMarkers(
        std::string message,
        geometry_msgs::Pose pose,
        std::string frame_id, 
        int index_offset, 
        bool add = true, 
        double r = 0.0, double g = 0.0, double b = 1.0, double lifetime = -1.0) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "text";
        marker.id = index_offset;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        if (add) {
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = pose.position.x;
            marker.pose.position.y = pose.position.y;
            marker.pose.position.z = pose.position.z + 0.6;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(0.0);
            marker.pose.orientation = quat;
            marker.text = message;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.3;
            marker.color.g = g;
            marker.color.r = r;
            marker.color.b = b;
            marker.color.a = 1.0;
        } else {
            marker.action = visualization_msgs::Marker::DELETE;
        }
        
        if (lifetime < 0.0)
            marker.lifetime = ros::Duration();
        else 
            marker.lifetime = ros::Duration(lifetime);
            
        return marker;
    }
    
    visualization_msgs::Marker getBoxMarker(
        geometry_msgs::Pose pose,
        std::string frame_id, 
        int index_offset, 
        bool add = true,
        double r = 0.0, double g = 1.0, double b = 0.0, double lifetime = -1.0) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube";
        marker.id = index_offset;   
        
        if (add) {
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = 0.1; 
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.7;
            
            pose.position.z += (marker.scale.z/2);
            marker.pose = pose;
        } else {
            marker.action = visualization_msgs::Marker::DELETE;
        }
        
        if (lifetime < 0.0)
            marker.lifetime = ros::Duration();
        else 
            marker.lifetime = ros::Duration(lifetime);
        
        return marker;
    }
    
}

class AddMarkers{
public:
    AddMarkers(ros::NodeHandle nh, ros::NodeHandle private_nh) 
        : nh_(nh), private_nh_(private_nh)
    {
        std::string odom_topic, objinfo_marker_topic, obj_marker_topic, picking_info_topic, carrying_info_topic, carrying_obj_marker_topic, robotinfo_marker_topic;
        
        private_nh_.param<std::string>("odom_topic", odom_topic, "odom");
        private_nh_.param<std::string>("object_info_marker_topic", objinfo_marker_topic, "object_info");
        private_nh_.param<std::string>("object_marker_topic", obj_marker_topic, "object");
        private_nh_.param<std::string>("carrying_obj_marker_topic", carrying_obj_marker_topic, "object_carrying");

        private_nh_.param<std::string>("picking_info_topic", picking_info_topic, "picking_info");
        private_nh_.param<std::string>("carrying_info_topic", carrying_info_topic, "carrying_info");
        private_nh_.param<std::string>("robotinfo_marker_topic", robotinfo_marker_topic, "robot_info");
        
        odom_sub_ = nh_.subscribe(odom_topic, 10, &AddMarkers::robotOdomCallback, this);
        picking_sub_ = nh_.subscribe(picking_info_topic, 10, &AddMarkers::pickingInfoCallback, this);
        carrying_sub_ = nh_.subscribe(carrying_info_topic, 1, &AddMarkers::carryingInfoCallback, this);
        
        objinfo_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(objinfo_marker_topic, 1);
        obj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(obj_marker_topic, 1);
        robotinfo_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(robotinfo_marker_topic, 1);
        carrying_obj_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(carrying_obj_marker_topic, 1);
    }
    
    void pickingInfoCallback(const pick_msgs::PickedItem msg) {
        
        current_item_ = msg;
        
        visualization_msgs::Marker text_marker, box_marker;
        if (msg.robot_status == "picking") {
            box_marker = utils::getBoxMarker(msg.pose.pose, msg.pose.header.frame_id, 10, true, 0.0, 0.0, 1.0);
            text_marker = utils::getTextMarkers("Picking Zone", msg.pose.pose, msg.pose.header.frame_id, 20, true, 0.0, 0.0, 1.0);
        } else if (msg.robot_status == "dropping") {
            box_marker = utils::getBoxMarker(msg.pose.pose, msg.pose.header.frame_id, 10, true, 1.0, 0.0, 0.0);  
            text_marker = utils::getTextMarkers("Dropping Zone", msg.pose.pose, msg.pose.header.frame_id, 20, true, 1.0, 0.0, 0.0);
        }

        ROS_INFO("Status: %s", msg.robot_status.c_str());
        obj_marker_pub_.publish(box_marker);
        objinfo_marker_pub_.publish(text_marker);
    }
    
    void carryingInfoCallback(const pick_msgs::PickedItem msg) {
                
        visualization_msgs::Marker box_marker;
        if (msg.robot_status == "carrying") {
            box_marker = utils::getBoxMarker(msg.pose.pose, msg.pose.header.frame_id, 40, true, 0.2, 0.2, 1.0, 1.0);
        }

        ROS_INFO("Status: %s", msg.robot_status.c_str());
        carrying_obj_marker_pub_.publish(box_marker);
    }
    
    void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
        
        double err_x = msg->pose.pose.position.x - current_item_.pose.pose.position.x;
        double err_y = msg->pose.pose.position.y - current_item_.pose.pose.position.y;
        double err_theta = fabs(utils::getYawFromQuaternion(msg->pose.pose.orientation) - utils::getYawFromQuaternion(current_item_.pose.pose.orientation));
        double err_dist = sqrtf(err_x * err_x + err_y * err_y);
                
        std::stringstream ss;
        ss << "Task: " << current_item_.robot_status << "\n";
        ss << "Err dist: " << std::setprecision(3) << err_dist;
        ss << ", theta: " << std::setprecision(3) << err_theta;
        
        geometry_msgs::Pose pose;
        pose.position.x = 0.3;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        
        visualization_msgs::Marker text_marker;
        text_marker = utils::getTextMarkers(ss.str(), pose, "base_footprint", 30, true, 1.0, 1.0, 1.0);
        
        robotinfo_marker_pub_.publish(text_marker);
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher objinfo_marker_pub_, obj_marker_pub_, carrying_obj_marker_pub_;
    ros::Publisher robotinfo_marker_pub_;
    ros::Subscriber picking_sub_, carrying_sub_;
    
    pick_msgs::PickedItem current_item_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh, private_nh("~");
    
    AddMarkers add_markers(nh, private_nh);
    ros::spin();
    
    return 0;
}
