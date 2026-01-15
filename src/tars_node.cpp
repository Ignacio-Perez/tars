#include <chrono>
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/polygon.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tars/tars.hpp"
#include "tars/ros2_interface.hpp"
#include "tars/msg/agents_msg.hpp"
#include "tars/srv/robot_goal_srv.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TarsNode : public rclcpp::Node 
{
public:
    TarsNode();
    void init();
private:
    void callback();
    void setRobotGoal(const std::shared_ptr<tars::srv::RobotGoalSrv::Request> req,
          std::shared_ptr<tars::srv::RobotGoalSrv::Response> res);

    rclcpp::TimerBase::SharedPtr callbackTimer;
    bool firstCall = true;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr nodesPub;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr edgesPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agentsVisPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nodesVisPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edgesVisPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forcesVisPub;
    rclcpp::Publisher<tars::msg::AgentsMsg>::SharedPtr agentsPub;
    rclcpp::Service<tars::srv::RobotGoalSrv>::SharedPtr service;
    std::vector<Ros2Interface> interfaces;
    builtin_interfaces::msg::Time currentTime;
    builtin_interfaces::msg::Time prevTime;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    double dt;
};

TarsNode::TarsNode()
: Node("tars_node") {}


void TarsNode::setRobotGoal(const std::shared_ptr<tars::srv::RobotGoalSrv::Request> req,
          std::shared_ptr<tars::srv::RobotGoalSrv::Response> res) {
    for (unsigned i=0;i<AGENTS.size();i++) {
        if (AGENTS[i]->getName()==req->id && AGENTS[i]->getType()==ROBOT) {
            Robot* robot = (Robot*)AGENTS[i];
            utils::Vector2d goal(req->gx,req->gy);
            robot->setGoal(goal);
            res->error=false;
            return;
        }
    }
    res->error=true;
    return;   
}

void TarsNode::init() {
    this->declare_parameter<std::string>("scenario","");
    std::string scenario = this->get_parameter("scenario").as_string();
    if (scenario.size()==0) {
        throw std::runtime_error("Empty scenario path");
    }
    RCLCPP_INFO(this->get_logger(), "Loading scenario '%s'", scenario.c_str()); 
    TARS.load(scenario); 
    RCLCPP_INFO(this->get_logger(), "Done!"); 
    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    for (unsigned i=0;i<AGENTS.size();i++) {
        if (AGENTS[i]->getType()==ROBOT) {
            interfaces.emplace_back((Robot*)AGENTS[i],this);
        }   
    }
    int period = std::round(1000.0 / TARS.getFreq());
    rclcpp::QoS latched(1);
    latched.transient_local();
    latched.reliable();
    nodesPub = this->create_publisher<geometry_msgs::msg::Polygon>("tars/nodes",latched);
    edgesPub = this->create_publisher<geometry_msgs::msg::Polygon>("tars/edges",latched);
    agentsVisPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tars/visualization/agents",1);
    nodesVisPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tars/visualization/nodes",1);
    edgesVisPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tars/visualization/edges",1);
    forcesVisPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("tars/visualization/forces",1);
    agentsPub = this->create_publisher<tars::msg::AgentsMsg>("tars/agents",1);
    service = this->create_service<tars::srv::RobotGoalSrv>("tars/robot_goal", std::bind(&TarsNode::setRobotGoal,this,std::placeholders::_1, std::placeholders::_2));
    callbackTimer = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&TarsNode::callback,this));  
}

void TarsNode::callback() {
    currentTime = this->get_clock()->now();
    if (firstCall) {
        dt = 0.0;
        firstCall = false;
        Ros2Interface::publishNodes(nodesPub);
        Ros2Interface::publishEdges(edgesPub);
    } else {
        dt = (rclcpp::Time(currentTime) - rclcpp::Time(prevTime)).seconds();
    }
    TARS.update(dt);    
    for (auto it = interfaces.begin(); it!= interfaces.end(); ++it) {
        it->publish(br,currentTime);
    }
    Ros2Interface::publishAgents(agentsPub,currentTime);
    Ros2Interface::publishNodesVisualization(nodesVisPub,currentTime);
    Ros2Interface::publishEdgesVisualization(edgesVisPub,currentTime);
    Ros2Interface::publishForcesVisualization(forcesVisPub,currentTime);
    Ros2Interface::publishAgentsVisualization(agentsVisPub,currentTime);
    prevTime = currentTime;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TarsNode>();
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
