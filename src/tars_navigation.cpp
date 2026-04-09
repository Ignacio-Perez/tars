
#include <string>
#include <chrono>
#include <list>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tars/navigation_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tars/tf2.hpp"
#include "tars/rrt.hpp"

using std::placeholders::_1;

enum State {INIT, COMPUTING_RRT, NAVIGATING};

class TarsNavigation : public rclcpp::Node 
{
public:
    TarsNavigation();
    void init();
private:

    void control();
    void publishPath();
    void mapReceivedCallback(const nav_msgs::msg::OccupancyGrid& msg);
    void odomReceivedCallback(const nav_msgs::msg::Odometry& msg);
    void goalReceivedCallback(const geometry_msgs::msg::PoseStamped& msg);

    double robotRadius;

    State robotState = INIT;
    utils::Vector2d robotPosition;
    utils::Vector2d robotInitialPosition;  
    utils::Angle robotYaw; 
    utils::Vector2d robotGoal;
    bool goalReceived = false;
    std::list<utils::Vector2d> path;
    RRT rrt;

    double computingTime;

    bool usingRRTStar;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pathPub;

    rclcpp::TimerBase::SharedPtr timer;

};


TarsNavigation::TarsNavigation()
: Node("tars_navigation") {}

void TarsNavigation::init() {
    
    robotState = INIT;
    goalReceived = false;
    path.clear();

    TF2.init(shared_from_this());

    declare_parameter<double>("node_freq",20.0);
    double nodeFreq = get_parameter("node_freq").as_double();

    computingTime = (1.0 / nodeFreq)*0.9;

    declare_parameter<std::string>("goal_topic","/goal_pose");
    std::string goalTopic = get_parameter("goal_topic").as_string();

    declare_parameter<std::string>("map_topic","/map");
    std::string mapTopic = get_parameter("map_topic").as_string();

    declare_parameter<std::string>("nav_map_topic","/nav_map");
    std::string navMapTopic = get_parameter("nav_map_topic").as_string();

    declare_parameter<std::string>("odom_topic","/tars/r09/odom");
    std::string odomTopic = get_parameter("odom_topic").as_string();

    declare_parameter<std::string>("path_topic","/tars/visualization/r09/path");
    std::string pathTopic = get_parameter("path_topic").as_string();

    declare_parameter<double>("robot_radius",0.2);
    robotRadius = get_parameter("robot_radius").as_double();

    declare_parameter<bool>("rrt_star",true);
    usingRRTStar = get_parameter("rrt_star").as_bool();

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    mapSub = create_subscription<nav_msgs::msg::OccupancyGrid>(mapTopic, qos, std::bind(&TarsNavigation::mapReceivedCallback, this, _1));
    odomSub = create_subscription<nav_msgs::msg::Odometry>(odomTopic,1,std::bind(&TarsNavigation::odomReceivedCallback, this, _1));
    goalSub = create_subscription<geometry_msgs::msg::PoseStamped>(goalTopic,1, std::bind(&TarsNavigation::goalReceivedCallback, this, _1));
    pathPub = create_publisher<visualization_msgs::msg::Marker>(pathTopic,1);

    mapPub = create_publisher<nav_msgs::msg::OccupancyGrid>(navMapTopic,qos);

    int period = std::round(1000.0 / nodeFreq);

    timer = create_wall_timer(std::chrono::milliseconds(period), std::bind(&TarsNavigation::control,this));  

}

void TarsNavigation::mapReceivedCallback(const nav_msgs::msg::OccupancyGrid& msg) {
    RCLCPP_INFO(get_logger(), "MAP RECEIVED");    
    NAV_MAP.setData(msg,robotRadius);
    mapPub->publish(NAV_MAP.getMap());
}

void TarsNavigation::odomReceivedCallback(const nav_msgs::msg::Odometry& msg) {
    bool success = TF2.transform(msg,robotPosition,robotYaw,"map");
    if (success && robotState == INIT && NAV_MAP.isInitiated()) {
        RCLCPP_INFO(get_logger(),"ODOM RECEIVED X = %f Y = %f",robotPosition.getX(),robotPosition.getY());
        robotState = COMPUTING_RRT;
        robotInitialPosition = robotPosition;
        RCLCPP_INFO(get_logger(),"State: COMPUTING_RRT");
        rrt.init(robotPosition,robotRadius, usingRRTStar);
    }   
}

void TarsNavigation::goalReceivedCallback(const geometry_msgs::msg::PoseStamped& msg) {
    if (robotState != COMPUTING_RRT) {
        return;
    }

    if (msg.header.frame_id != "map") {
        RCLCPP_WARN(get_logger(),"Invalid Goal frame_id '%s'",msg.header.frame_id.c_str());
        return;
    }
    
    robotGoal.set(msg.pose.position.x, msg.pose.position.y);
    goalReceived = true;
    RCLCPP_INFO(get_logger(),"GOAL RECEIVED X = %f Y = %f",robotGoal.getX(),robotGoal.getY());
}

void TarsNavigation::control() {
    rclcpp::Time startTime = get_clock()->now();
    while (robotState == COMPUTING_RRT && (get_clock()->now() - startTime).seconds() < computingTime) {
        rrt.iterate();
    }
    if (robotState == COMPUTING_RRT && goalReceived) {
        rrt.getPath(robotGoal,path);
    }
    if (robotState == COMPUTING_RRT || robotState == NAVIGATING) {
        publishPath(); 
    }
}

void TarsNavigation::publishPath() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.id = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;      
    marker.pose.orientation.z = 0;      
    marker.pose.orientation.w = 1.0;                
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;   
    marker.action = visualization_msgs::msg::Marker::ADD;    
    geometry_msgs::msg::Point prev,next;
    prev.x = robotInitialPosition.getX();
    prev.y = robotInitialPosition.getY();
    prev.z = 0;
    next.z = 0;
    for (auto it = path.begin();it!=path.end();++it) {
        next.x = it->getX();
        next.y = it->getY();
        marker.points.push_back(prev);
        marker.points.push_back(next);
        prev = next;
    }
    pathPub->publish(marker);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TarsNavigation>();
  try {
    node->init();
    rclcpp::spin(node);
  } catch(const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());    
    rclcpp::shutdown();
    return 1;
  } catch(...) {
    RCLCPP_FATAL(node->get_logger(), "unknown error, node aborted");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}