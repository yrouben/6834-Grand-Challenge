#include <pluginlib/class_list_macros.h>
#include "sampling_planner.h"
#include "sampling_path_planning/MapInfo.h"
#include "sampling_path_planning/PathEndPoints.h"
#include "nav_msgs/Path.h"

PLUGINLIB_EXPORT_CLASS(sampling_planner::SamplingPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    ROS_INFO_STREAM("received");
}

namespace sampling_planner {

SamplingPlanner::SamplingPlanner()
: costmap_ros_(NULL),costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), initialized_(false){}

SamplingPlanner::SamplingPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
: costmap_ros_(NULL),costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), initialized_(false){
initialize(name, costmap_ros);
}

void SamplingPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){



    if(!initialized_){
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle n;
        ros::NodeHandle nh("~/" + name);
        ros::Publisher obstacles_pub = n.advertise<sampling_path_planning::MapInfo>("map_info", 1000);
        ros::Publisher endpts_pub_= n.advertise<sampling_path_planning::PathEndPoints>("start_and_end", 1000);
        ros::Subscriber path_sub_ = n.subscribe("PRM_path", 1000, pathCallback);

        //try{
            costmap_converter_ = costmap_converter_loader_.createInstance("costmap_converter::CostmapToPolygonsDBSMCCH");
            std::string converter_name = costmap_converter_loader_.getName("costmap_converter::CostmapToPolygonsDBSMCCH");
            // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
            boost::replace_all(converter_name, "::", "/");
            costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
            costmap_converter_->setCostmap2D(costmap_);
        
            costmap_converter_-> compute();

            ROS_INFO_STREAM("Costmap conversion plugin " << "costmap_converter_plugin" << " loaded.");        
        //}
        //catch(pluginlib::PluginlibException& ex){
        //    ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        //    costmap_converter_.reset();
        //}
        costmap_converter::PolygonContainerConstPtr polygons = costmap_converter_->getPolygons();
        const double originX = costmap_ -> getOriginX();
        const double originY = costmap_ -> getOriginY();
        const double sizeX = costmap_ -> getSizeInMetersX();
        const double sizeY = costmap_ -> getSizeInMetersY();
        
        // make msg for mapinfo and publish
        sampling_path_planning::MapInfo mapInfo;
        mapInfo.polygons = *polygons;
        mapInfo.originX = originX;
        mapInfo.originY = originY;
        mapInfo.lenX = sizeX;
        mapInfo.lenY = sizeY;

        obstacles_pub.publish(mapInfo);
        
            
        initialized_ = true;
    }   
    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

bool SamplingPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    // Need to publish the endpoints
    // need to subscribe to path 
    sampling_path_planning::PathEndPoints endPts;
    endPts.startpose = start.pose;
    endPts.goalpose = goal.pose;

    endpts_pub_.publish(endPts);

    // This is a dummer makeplan code. we just need to call our path planner here.
    // custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);
    plan.push_back(start);
    for (int i=0; i<20; i++){
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
        new_goal.pose.position.x = -2.5+(0.05*i);
        new_goal.pose.position.y = -3.5+(0.05*i);

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    }

    plan.push_back(goal);
    return true;
}
};
