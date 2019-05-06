#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <pick_msgs/PickedItem.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <boost/thread.hpp>
#include <vector>
#include <algorithm>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point2f {
    float x;
    float y;
    Point2f(float _x, float _y) {
        x = _x;
        y = _y;
    }
};

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
        
        std::string picking_info_topic, carrying_info_topic;
        private_nh_.param<std::string>("picking_info_topic", picking_info_topic, "picking_info");
        private_nh_.param<std::string>("carrying_info_topic", carrying_info_topic, "carrying_info");
        
        private_nh_.param<std::string>("action_name", action_name_, "move_base");
        private_nh_.param<std::string>("goal_pose_frame", goal_pose_frame_, "map");
 
        std::string map_topic;
        private_nh_.param<std::string>("map_topic", map_topic, "/map");

        picking_pub_= nh_.advertise<pick_msgs::PickedItem>(picking_info_topic, 1);
        carrying_pub_ = nh_.advertise<pick_msgs::PickedItem>(carrying_info_topic, 10);
        map_sub_ = nh_.subscribe(map_topic, 1, &PickObjects::mapCallback, this);
        
    }
    
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
        map_ = *msg;
        ROS_INFO("Received map: %d x %d, resol: %.3f, Origin: %.3f, %.3f", 
                 msg->info.width, msg->info.height, msg->info.resolution,
                 msg->info.origin.position.x,
                 msg->info.origin.position.y
                );
    }
    
    void feedbackActionCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {   
        if (current_status_ == "dropping") {
            geometry_msgs::Pose pose;
            pose.position.y = 0.0;
            pose.position.x = 0.3;
            pose.position.z = 0.0;
            pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            pick_msgs::PickedItem msg = this->getPickingItem("base_footprint", "carrying", pose);
            carrying_pub_.publish(msg);
        }
    }
    
    void doneActionCallback(const actionlib::SimpleClientGoalState& state) {
        ROS_INFO("Done: %s [%s]", state.toString().c_str(), state.getText().c_str());
    }
    
    void run() {
        
        ROS_WARN("Awaiting for map data ...");
        ros::Rate rate(10.0);
        while(map_.data.empty()) {
            rate.sleep();
            ros::spinOnce();
        }
        ROS_INFO("Map is ready.");
        
        ac_ = new MoveBaseClient(action_name_, true);
        while(!ac_->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the '%s' action server to come up", action_name_.c_str());
        }
        
        this->setCandidateMap();
        
        std::string robot_status[2] = {"picking", "dropping"};
        int status_index = 0;
        
        while (ros::ok()) {
            
            current_status_ = robot_status[status_index];
            
            ROS_INFO("Awaiting for '%s'", robot_status[status_index].c_str());
            ros::Duration(awaiting_interval_time_).sleep();
            
            ros::Time t_begin = ros::Time::now();
            move_base_msgs::MoveBaseGoal goal = this->getRandomGoalFromMap();
            
            pick_msgs::PickedItem msg1 = this->getPickingItem(goal_pose_frame_, robot_status[status_index], goal.target_pose.pose);
            picking_pub_.publish(msg1);
            status_index = (status_index + 1) % 2;
            
            ac_->sendGoal(
                goal,
                boost::bind(&PickObjects::doneActionCallback, this, _1),
                MoveBaseClient::SimpleActiveCallback(),
                boost::bind(&PickObjects::feedbackActionCallback, this, _1)
            );
   
            ac_->waitForResult();
            
            double duration = (ros::Time::now() - t_begin).toSec();
            if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_WARN("Task duration %.3lf sec, Successful.", duration);
            } else {
                ROS_WARN("Task duration %.3lf sec, Failed !", duration);                
            }
            
            rate.sleep();
            ros::spinOnce();
        }
    }
    
    void setCandidateMap() {
        nav_msgs::OccupancyGrid src_map = map_;
  
        int obstacle_num = 0;
        for (int i=0; i<map_.info.width; i++) {
            for (int j=0; j<map_.info.height; j++) {
                int index = j * map_.info.width + i;
                bool is_obstacle_cell = ((int)src_map.data[index] > (int)(100.0 * occ_thresh_));
                if (is_obstacle_cell) {
                    this->setPotentialField(i, j, 2.0);    // field_size in meter
                    obstacle_num ++;
                }
            }
        }
        ROS_WARN("Number of obstacles: %d", obstacle_num);
        
        free_cells_.clear();
        for (int i=0; i<map_.info.width; i++) {
            for (int j=0; j<map_.info.height; j++) {
                int index = j * map_.info.width + i;
                bool is_free_cell = (int)map_.data[index] == 0;
                if(is_free_cell) {
                    map_.data[index] = 0;
                    Point2f pt(
                        (float)(i) * map_.info.resolution + map_.info.origin.position.x,
                        (double)(j) * map_.info.resolution + map_.info.origin.position.y
                    );
                    free_cells_.push_back(pt);
                }
            }
        }
    }
    
    pick_msgs::PickedItem getPickingItem(std::string frame_id, std::string action, geometry_msgs::Pose pose) {
        pick_msgs::PickedItem msg;
        msg.robot_status = action;
        double yaw = utils::getRandDouble() * M_PI;
        msg.pose.header.frame_id = frame_id;
        msg.pose.header.stamp = ros::Time::now();
        msg.pose.pose = pose;
        return msg;
    }
    
    void setPotentialField(int x, int y, double field_size) {        
        int patch_size = (int)(field_size / map_.info.resolution);
        for (int i=x-patch_size/2; i<x+patch_size/2; i++) {
            for (int j=y-patch_size/2; j<y+patch_size/2; j++) {
                int index = j * map_.info.width + i;
                if (index>=0 and index<map_.info.width*map_.info.height) {
                    map_.data[index] = 100;
                }
            }
        }
    }
    
    move_base_msgs::MoveBaseGoal getRandomGoalFromMap() {
        move_base_msgs::MoveBaseGoal goal;
        if ((int)free_cells_.size() > 10) {
            int index = rand() % (int)free_cells_.size();
            goal.target_pose.header.frame_id = goal_pose_frame_;
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = free_cells_[index].x;
            goal.target_pose.pose.position.y = free_cells_[index].y;
            double yaw = (double)(rand() % 100) / 100.0f * 2 * M_PI;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        } else {
            ROS_WARN("No available free cell for setting goal");
        }
        return goal;
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher picking_pub_, carrying_pub_;
    ros::Subscriber map_sub_;

    double awaiting_interval_time_;
    
    MoveBaseClient* ac_;
    nav_msgs::OccupancyGrid map_;
    double occ_thresh_;
    std::vector<Point2f> free_cells_;
    
    std::string action_name_;
    std::string goal_pose_frame_;
    std::string current_status_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle nh, private_nh("~");
    
    PickObjects picker(nh, private_nh);
    picker.run();
    
    return 0;
}
