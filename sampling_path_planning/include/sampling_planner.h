#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <costmap_converter/costmap_converter_interface.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

using std::string;

#ifndef SAMPLING_PLANNER_CPP
#define SAMPLING_PLANNER_CPP

namespace sampling_planner {

class SamplingPlanner : public nav_core::BaseGlobalPlanner {

private:
	bool initialized_;
    ros::Publisher endpts_pub_;
    ros::Subscriber path_sub_;
    costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
	pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
	boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
public:

    SamplingPlanner();
    SamplingPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    };
};

#endif
