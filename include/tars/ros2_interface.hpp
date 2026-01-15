/*
MIT License

Copyright (c) 2026 Ignacio Pérez Hurtado de Mendoza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _ROS2_INTERFACE_HPP_
#define _ROS2_INTERFACE_HPP_

#include <cmath>
#include <unordered_set>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tars/msg/agents_msg.hpp"
#include "tars/msg/agent_msg.hpp"

#include "tars/tars.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

class Ros2Interface
{
public:
	Ros2Interface(Robot* robot, rclcpp::Node* node);
	void publish(
		const std::shared_ptr<tf2_ros::TransformBroadcaster>& br,
		const builtin_interfaces::msg::Time& currentTime);

	static void publishNodes(const rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr& nodesPub);
	static void publishEdges(const rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr& edgesPub);

	static void publishAgents(
		const rclcpp::Publisher<tars::msg::AgentsMsg>::SharedPtr& agentsPub, 
		const builtin_interfaces::msg::Time& currentTime);
	
	static void publishForcesVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& forcesPub,
		const builtin_interfaces::msg::Time& currentTime);

	static void publishNodesVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& nodesPub,
		const builtin_interfaces::msg::Time& currentTime);

	static void publishEdgesVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& edgesPub,
		const builtin_interfaces::msg::Time& currentTime);

	static void publishAgentsVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& agentsPub,
		const builtin_interfaces::msg::Time& currentTime);

private:
	static void agentToMsg(const Agent& agent, tars::msg::AgentMsg& msg);
	static std_msgs::msg::ColorRGBA getColor(double r, double g, double b, double a);
	static void publishForceMarker(const builtin_interfaces::msg::Time& currentTime,
					unsigned index, 
					const std_msgs::msg::ColorRGBA& color, 
					const utils::Vector2d& force, 
					const utils::Vector2d& position,
					visualization_msgs::msg::MarkerArray& markers);

	void cmdVelReceived(const geometry_msgs::msg::Twist& cmd_vel);

	Robot* robot;
	rclcpp::Node* node;
	sensor_msgs::msg::LaserScan scan;
	utils::Vector2d lastScanPosition;
	utils::Angle lastScanYaw;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trackingVisPub;
	rclcpp::Publisher<tars::msg::AgentsMsg>::SharedPtr trackingPub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
};


inline
Ros2Interface::Ros2Interface(Robot* robot, rclcpp::Node* node)
: robot(robot), node(node) {

	trackingVisPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("tars/visualization/"+robot->getName()+"/agents_tracking",1);
	trackingPub = node->create_publisher<tars::msg::AgentsMsg>("tars/"+robot->getName()+"/agents_tracking",1);
	odomPub = node->create_publisher<nav_msgs::msg::Odometry>("tars/"+robot->getName()+"/odom",1);
	scanPub = node->create_publisher<sensor_msgs::msg::LaserScan>("tars/"+robot->getName()+"/scan",1);
	cmdVelSub = node->create_subscription<geometry_msgs::msg::Twist>(
      "tars/"+robot->getName()+"/cmd_vel", 1, std::bind(&Ros2Interface::cmdVelReceived, this, _1));

	scan.header.frame_id = robot->getName() + "/base_link";
	scan.angle_min = -M_PI;
	scan.angle_max = M_PI;
	scan.angle_increment = robot->getScanAngleIncrement();
	scan.time_increment = 0;
	scan.scan_time = 1.0/TARS.getFreq();
	scan.range_min = robot->getRadius();
	scan.range_max = robot->getScanRange();

}

inline
void Ros2Interface::cmdVelReceived(const geometry_msgs::msg::Twist& cmd_vel) {
	robot->setCmdVel(cmd_vel.linear.x,cmd_vel.angular.z);
}

inline
void Ros2Interface::agentToMsg(const Agent& agent, tars::msg::AgentMsg& msg) {
	msg.id = agent.getName();
	msg.type = (unsigned)(agent.getType());
	msg.radius = agent.getRadius();
	msg.position.x = agent.getPosition().getX();
	msg.position.y = agent.getPosition().getY();
	msg.yaw = agent.getYaw().toRadian();
	msg.velocity.x = agent.getVelocity().getX();
	msg.velocity.y = agent.getVelocity().getY();
	msg.has_forces = agent.hasForces();
	msg.goal.x = agent.getGoal().getX();
	msg.goal.y = agent.getGoal().getY();
	msg.forces.desired_force.x = agent.getForces().desiredForce.getX();
	msg.forces.desired_force.y = agent.getForces().desiredForce.getY();
	msg.forces.obstacle_force.x = agent.getForces().obstacleForce.getX();
	msg.forces.obstacle_force.y = agent.getForces().obstacleForce.getY();
	msg.forces.social_force.x = agent.getForces().socialForce.getX();
	msg.forces.social_force.y = agent.getForces().socialForce.getY();
	msg.forces.global_force.x = agent.getForces().globalForce.getX();
	msg.forces.global_force.y = agent.getForces().globalForce.getY();
}

inline
std_msgs::msg::ColorRGBA Ros2Interface::getColor(double r, double g, double b, double a) {
	std_msgs::msg::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}

inline
void Ros2Interface::publish(
	const std::shared_ptr<tf2_ros::TransformBroadcaster>& br,
	const builtin_interfaces::msg::Time& currentTime) {
	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = currentTime;
	transform.header.frame_id = "map";
	transform.child_frame_id = robot->getName() + "/odom";
	transform.transform.translation.x = robot->getInitialPosition().getX();
	transform.transform.translation.y = robot->getInitialPosition().getY();
	transform.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, robot->getInitialYaw().toRadian());
	q.normalize();
	transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
	br->sendTransform(transform);
	
	double x = robot->getPosition().getX() - robot->getInitialPosition().getX();
	double y = robot->getPosition().getY() - robot->getInitialPosition().getY();
	double a = -robot->getInitialYaw().toRadian();
	double c = std::cos(a);
	double s = std::sin(a);
	transform.header.frame_id = robot->getName() + "/odom";
	transform.child_frame_id = robot->getName() + "/base_link";
	transform.transform.translation.x = x*c - y*s;
	transform.transform.translation.y = x*s + y*c; 
	q.setRPY(0.0, 0.0, (robot->getYaw() - robot->getInitialYaw()).toRadian());
	q.normalize();
	transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
	br->sendTransform(transform);
	
	nav_msgs::msg::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = robot->getName() + "/odom";
	odom.child_frame_id =  robot->getName() + "/base_link";
	odom.pose.pose.position.x = transform.transform.translation.x;
	odom.pose.pose.position.y = transform.transform.translation.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = transform.transform.rotation;
	odom.twist.twist.linear.x = robot->getLinVel();
	odom.twist.twist.linear.y = 0.0; 
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = robot->getAngVel();
	odomPub->publish(odom);
	scan.header.stamp = currentTime;
	if (lastScanPosition != robot->getPosition() || lastScanYaw != robot->getYaw()) {	
		lastScanPosition = robot->getPosition();
		lastScanYaw =robot->getYaw();
		scan.ranges = robot->getScan();
	}
	scanPub->publish(scan);
	tars::msg::AgentsMsg msg;
	msg.header.frame_id = "map";
	msg.header.stamp = currentTime;
	msg.size = robot->getAgentsDetected().size();
	msg.agents.resize(msg.size);
	std::unordered_set<std::string> names;
	for (unsigned i=0;i<msg.size;i++) {
		agentToMsg(*AGENTS[robot->getAgentsDetected()[i]],msg.agents[i]);
		if (AGENTS[robot->getAgentsDetected()[i]]->getName() != robot->getName()) {
			names.insert(AGENTS[robot->getAgentsDetected()[i]]->getName());
		}
	}
	trackingPub->publish(msg);
	visualization_msgs::msg::MarkerArray markers;
	for (unsigned i=0;i<AGENTS.size();i++) {
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = "map";
	    marker.header.stamp = currentTime;
	    if (names.count(AGENTS[i]->getName())>0) {
			marker.action = visualization_msgs::msg::Marker::ADD;	
		} else {
			marker.action = visualization_msgs::msg::Marker::DELETE;

		}
		marker.color.a = 0.5;	
		marker.id = i;
		marker.type = visualization_msgs::msg::Marker::CYLINDER;
		marker.color.r = 1.0;
	    marker.color.g = 0;
	    marker.color.b = 0;
	    marker.scale.x = AGENTS[i]->getRadius()*2;
		marker.scale.y = AGENTS[i]->getRadius()*2;
		if (AGENTS[i]->getType()==HUMAN) {
			marker.scale.z = 1.2;
		} else {
			marker.scale.z = 0.3;
		}
		marker.pose.position.x = AGENTS[i]->getPosition().getX();
		marker.pose.position.y = AGENTS[i]->getPosition().getY();
		marker.pose.position.z = marker.scale.z / 2;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;		
		marker.pose.orientation.z = 0;		
		marker.pose.orientation.w = 1.0;			
	    markers.markers.push_back(marker);
	}
	trackingVisPub->publish(markers);
}

inline
void Ros2Interface::publishForceMarker(const builtin_interfaces::msg::Time& currentTime,
					unsigned index, 
					const std_msgs::msg::ColorRGBA& color, 
					const utils::Vector2d& force, 
					const utils::Vector2d& position,
					visualization_msgs::msg::MarkerArray& markers)  {
	tf2::Quaternion q;
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = currentTime;
	marker.ns = "forces";
	marker.id = index;
	marker.action = force.norm()>1e-4 ?0:2;
	marker.color = color;
	marker.lifetime = rclcpp::Duration::from_seconds(1.0);
	marker.scale.x = std::max(1e-4,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = position.getX();
	marker.pose.position.y = position.getY();
	marker.pose.position.z = 0;
	q.setRPY(0, 0, force.angle().toRadian());
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();		
	marker.pose.orientation.z = q.z();		
	marker.pose.orientation.w = q.w();		
	markers.markers.push_back(marker);
}

inline
void Ros2Interface::publishAgents(
	const rclcpp::Publisher<tars::msg::AgentsMsg>::SharedPtr& agentsPub, 
	const builtin_interfaces::msg::Time& currentTime) {
	tars::msg::AgentsMsg msg;
	msg.header.frame_id = "map";
	msg.header.stamp = currentTime;
	msg.size = AGENTS.size();
	msg.agents.resize(AGENTS.size());
	for (unsigned i=0;i<AGENTS.size();i++) {
		agentToMsg(*AGENTS[i],msg.agents[i]);
	}
	agentsPub->publish(msg);
}

inline
void Ros2Interface::publishNodes(const rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr& nodesPub) {
	geometry_msgs::msg::Polygon polygon;
	for (auto it = GRAPH.getNodes().begin(); it!= GRAPH.getNodes().end(); ++it) {
		geometry_msgs::msg::Point32 p;
		p.x = it->second.getX();
		p.y = it->second.getY();
		p.z = 0;
		polygon.points.push_back(p);
	}
	nodesPub->publish(polygon);
}

inline
void Ros2Interface::publishEdges(const rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr& edgesPub) {
	geometry_msgs::msg::Polygon polygon;
	for (auto it = GRAPH.getEdges().begin(); it!= GRAPH.getEdges().end(); ++it) {
		geometry_msgs::msg::Point32 p;
		p.x = GRAPH[it->first.getSrc()].getX();
		p.y = GRAPH[it->first.getSrc()].getY();
		p.z = 0;
		polygon.points.push_back(p);
		p.x = GRAPH[it->first.getDst()].getX();
		p.y = GRAPH[it->first.getDst()].getY();
		polygon.points.push_back(p);
	}
	edgesPub->publish(polygon);
}



inline
void Ros2Interface::publishForcesVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& forcesPub,
		const builtin_interfaces::msg::Time& currentTime) {
	visualization_msgs::msg::MarkerArray markers;
	unsigned counter=0;
	for (unsigned i=0;i<AGENTS.size();i++) {
		if (!AGENTS[i]->hasForces()) {
			continue;
		}
		publishForceMarker(currentTime,counter++,getColor(0,0,1,1),AGENTS[i]->getForces().obstacleForce,AGENTS[i]->getPosition(),markers);
		publishForceMarker(currentTime,counter++,getColor(0,1,1,1),AGENTS[i]->getForces().socialForce,AGENTS[i]->getPosition(),markers);
		publishForceMarker(currentTime,counter++,getColor(1,0,0,1),AGENTS[i]->getForces().desiredForce,AGENTS[i]->getPosition(),markers);
		publishForceMarker(currentTime,counter++,getColor(1,1,0,1),AGENTS[i]->getVelocity(),AGENTS[i]->getPosition(),markers);	
	}
	forcesPub->publish(markers);
}


inline
void Ros2Interface::publishNodesVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& nodesPub,
		const builtin_interfaces::msg::Time& currentTime) {
	visualization_msgs::msg::MarkerArray markers;
	unsigned counter = 0;
	for (auto it = GRAPH.getNodes().begin(); it!= GRAPH.getNodes().end(); ++it) {
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = "map";
	    marker.header.stamp = currentTime;
		marker.action = 0;
		marker.color.a = 1.0;	
		marker.id = counter++;
		marker.type = visualization_msgs::msg::Marker::SPHERE;
		marker.color.r = 1.0;
	    marker.color.g = 0;
	    marker.color.b = 0;
	    marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.pose.position.x = it->second.getX();
		marker.pose.position.y = it->second.getY();
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;		
		marker.pose.orientation.z = 0;		
		marker.pose.orientation.w = 1.0;
		markers.markers.push_back(marker);
		marker.id = counter++;
	    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	    marker.color.r = 1.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0;
	   	marker.pose.position.z = 0.2;
		marker.text = it->first;
        markers.markers.push_back(marker);
	}
	nodesPub->publish(markers);

}

inline
void Ros2Interface::publishEdgesVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& edgesPub,
		const builtin_interfaces::msg::Time& currentTime) {
	visualization_msgs::msg::MarkerArray markers;
	unsigned counter=0;
	double max = GRAPH.getMaxCost();
	double min = GRAPH.getMinCost();
	for (auto it = GRAPH.getEdges().begin(); it!= GRAPH.getEdges().end(); ++it) {
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = currentTime;
		marker.id = counter++;	
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;		
		marker.pose.orientation.z = 0;		
		marker.pose.orientation.w = 1.0;				
		marker.type = visualization_msgs::msg::Marker::LINE_LIST;
		marker.scale.x = 0.05;
		marker.color.a = (it->second - min)/(max - min);
		marker.color.r = 0;
		marker.color.g = 0;
		marker.color.b = 1.0;
		geometry_msgs::msg::Point p;
		p.x = GRAPH[it->first.getSrc()].getX();
		p.y = GRAPH[it->first.getSrc()].getY();
		p.z = 0;
		marker.points.push_back(p);
		p.x = GRAPH[it->first.getDst()].getX();
		p.y = GRAPH[it->first.getDst()].getY();
		marker.points.push_back(p);
		if (marker.color.a>0) {
			marker.action = visualization_msgs::msg::Marker::ADD;	
		} else {
			marker.action = visualization_msgs::msg::Marker::DELETE;

		}
		markers.markers.push_back(marker);
	}
	edgesPub->publish(markers);

}

inline
void Ros2Interface::publishAgentsVisualization(
		const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& agentsPub,
		const builtin_interfaces::msg::Time& currentTime) {
	visualization_msgs::msg::MarkerArray markers;
	unsigned counter = 0;
	for (unsigned i=0;i<AGENTS.size();i++) {
		visualization_msgs::msg::Marker marker;
		tf2::Quaternion q;
		marker.header.frame_id = "map";
	    marker.header.stamp = currentTime;
		marker.action = 0;
		marker.color.a = 1.0;	
		marker.id = counter++;
		double scale = AGENTS[i]->getRadius()*2;
		if (AGENTS[i]->getType()==HUMAN) {
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.color.r = 1.0;
	        marker.color.g = 0;
	        marker.color.b = 0;
	        marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.3;
			marker.pose.position.x = AGENTS[i]->getPosition().getX();
			marker.pose.position.y = AGENTS[i]->getPosition().getY();
			marker.pose.position.z = 0.735;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;		
			marker.pose.orientation.z = 0;		
			marker.pose.orientation.w = 1.0;			
	        markers.markers.push_back(marker);

	        marker.id = counter++;
	    	marker.color.r = 0;
	        marker.color.b = 1.0;
	        marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.15;
			marker.pose.position.z = 0.51;
			markers.markers.push_back(marker);

			marker.id = counter++;
	    	double angle = AGENTS[i]->getYaw().toRadian() + M_PI/2;
	        marker.pose.position.x = AGENTS[i]->getPosition().getX() + std::cos(angle) * 0.06;
	        marker.pose.position.y = AGENTS[i]->getPosition().getY() + std::sin(angle) * 0.06;
	        marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.4;
			marker.pose.position.z = 0.25;
			markers.markers.push_back(marker);

			marker.id = counter++;
	    	angle = AGENTS[i]->getYaw().toRadian() - M_PI/2;
	        marker.pose.position.x = AGENTS[i]->getPosition().getX() + std::cos(angle) * 0.06;
	        marker.pose.position.y = AGENTS[i]->getPosition().getY() + std::sin(angle) * 0.06;
	        marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.4;
			marker.pose.position.z = 0.25;
			markers.markers.push_back(marker);

			marker.id = counter++;
	    	angle = AGENTS[i]->getYaw().toRadian() + M_PI/2;
	        marker.pose.position.x = AGENTS[i]->getPosition().getX() + std::cos(angle) * 0.15;
	        marker.pose.position.y = AGENTS[i]->getPosition().getY() + std::sin(angle) * 0.15;
	        marker.color.r = 1.0;
	        marker.color.b = 0;
	        marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.3;
			marker.pose.position.z = 0.735;
			markers.markers.push_back(marker);

			marker.id = counter++;
	    	angle = AGENTS[i]->getYaw().toRadian() - M_PI/2;
	        marker.pose.position.x = AGENTS[i]->getPosition().getX() + std::cos(angle) * 0.15;
	        marker.pose.position.y = AGENTS[i]->getPosition().getY() + std::sin(angle) * 0.15;
	        markers.markers.push_back(marker);

			marker.id = counter++;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
	    	angle = AGENTS[i]->getYaw().toRadian() + M_PI/2;
	        marker.pose.position.x = AGENTS[i]->getPosition().getX() + std::cos(angle) * 0.15;
	        marker.pose.position.y = AGENTS[i]->getPosition().getY() + std::sin(angle) * 0.15;
	        marker.color.r = 0.996078431;
	        marker.color.g = 0.764705882;
	        marker.color.b = 0.674509804;
	        marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.pose.position.z = 0.55;
			markers.markers.push_back(marker);

			marker.id = counter++;
	    	angle = AGENTS[i]->getYaw().toRadian() - M_PI/2;
	        marker.pose.position.x = AGENTS[i]->getPosition().getX() + std::cos(angle) * 0.15;
	        marker.pose.position.y = AGENTS[i]->getPosition().getY() + std::sin(angle) * 0.15;
	        markers.markers.push_back(marker);			

			marker.id = counter++;
	        marker.type = visualization_msgs::msg::Marker::CYLINDER;
	        marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.3;
			marker.pose.position.x = AGENTS[i]->getPosition().getX();
			marker.pose.position.y = AGENTS[i]->getPosition().getY();
			marker.pose.position.z = 0.95;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;		
			marker.pose.orientation.z = 0;		
			marker.pose.orientation.w = 1.0;			
	        markers.markers.push_back(marker);

	        marker.type = visualization_msgs::msg::Marker::SPHERE;
	        marker.id = counter++;
	        marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.3;
			marker.pose.position.z = 1.05;
			markers.markers.push_back(marker);

			marker.id = counter++;
	    	marker.type = visualization_msgs::msg::Marker::CYLINDER;
	    	marker.color.r = 0;
	    	marker.color.g = 0;
	    	marker.color.b = 0;
	    	marker.scale.x = scale;
			marker.scale.y = scale;
			marker.scale.z = 0.01;
			marker.pose.position.z = 0;
			markers.markers.push_back(marker);			   

			marker.id = counter++;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
	    	marker.scale.x = 0.05;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;
			utils::Angle a = utils::Angle::fromDegree(25);
			marker.pose.position.x = AGENTS[i]->getPosition().getX() + (AGENTS[i]->getYaw()+a).cos() * 0.15;
	    	marker.pose.position.y = AGENTS[i]->getPosition().getY() + (AGENTS[i]->getYaw()+a).sin() * 0.15;
			marker.pose.position.z = 1.05;
			markers.markers.push_back(marker);			   

			marker.id = counter++;
		  
			marker.pose.position.x = AGENTS[i]->getPosition().getX() + (AGENTS[i]->getYaw()-a).cos() * 0.15;
	    	marker.pose.position.y = AGENTS[i]->getPosition().getY() + (AGENTS[i]->getYaw()-a).sin() * 0.15;
			marker.pose.position.z = 1.05;
			markers.markers.push_back(marker);	

		   	marker.id = counter++;
	       	marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	       	marker.color.r = 1.0;
	       	marker.color.g = 1.0;
	       	marker.color.b = 0;
	       	marker.pose.position.x = AGENTS[i]->getPosition().getX();
			marker.pose.position.y = AGENTS[i]->getPosition().getY();
			marker.pose.position.z = 1.3;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;		
			marker.pose.orientation.z = 0;		
			marker.pose.orientation.w = 1;	
			
			marker.scale.z = 0.2;
			marker.text = AGENTS[i]->getName();
			markers.markers.push_back(marker);		

		} else {
			scale -= 0.05;
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.color.r = 0.0;
	        marker.color.g = 1.0;
	        marker.color.b = 0.0;
			marker.scale.x = scale;
	        marker.scale.y = scale;
	        marker.scale.z = 0.2;
			marker.pose.position.x = AGENTS[i]->getPosition().getX();
			marker.pose.position.y = AGENTS[i]->getPosition().getY();
			marker.pose.position.z = 0.1;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;		
			marker.pose.orientation.z = 0;		
			marker.pose.orientation.w = 1;				
			markers.markers.push_back(marker);
			marker.id = counter++;
			marker.color.g = 0.0;
	        marker.color.b = 1.0;
	        marker.scale.x = 0.1;
	        marker.scale.y = 0.1;
	        marker.scale.z = 0.05;
	        double angle = AGENTS[i]->getYaw().toRadian() + M_PI/2;
	        marker.pose.position.x += std::cos(angle) * scale/2;
	        marker.pose.position.y += std::sin(angle) * scale/2;
	        marker.pose.position.z -= 0.05;
	        q.setRPY(0, M_PI/2, AGENTS[i]->getYaw().toRadian()-M_PI/2);
	        marker.pose.orientation.x = q.x();
			marker.pose.orientation.y = q.y();		
			marker.pose.orientation.z = q.z();		
			marker.pose.orientation.w = q.w();			
	        markers.markers.push_back(marker);
	        marker.id = counter++;
		    marker.pose.position.x -= std::cos(angle) * scale;
	        marker.pose.position.y -= std::sin(angle) * scale;
	        markers.markers.push_back(marker);
	        marker.id = counter++;
	        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	        
	        marker.pose.position.x = AGENTS[i]->getPosition().getX();
			marker.pose.position.y = AGENTS[i]->getPosition().getY();
			marker.pose.position.z = 0.3;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;		
			marker.pose.orientation.z = 0;		
			marker.pose.orientation.w = 1;	
			marker.scale.z = 0.3;
			marker.text = AGENTS[i]->getName();
			markers.markers.push_back(marker);
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.id = counter++;
			marker.color.r = 1.0;
	        marker.color.b = 0.0;
			marker.scale.x = 0.05;
	        marker.scale.y = 0.05;
	        marker.scale.z = 0.01;
			marker.pose.position.x += AGENTS[i]->getYaw().cos() * scale * 0.4;
			marker.pose.position.y += AGENTS[i]->getYaw().sin() * scale * 0.4;
			marker.pose.position.z = 0.2;
			marker.text = "";
			markers.markers.push_back(marker);
		}
	}
	agentsPub->publish(markers);

}


/*
class ROSInterface
{
public:
	ROSInterface(Robot* robot, ros::NodeHandle& n, ros::NodeHandle& pn);
	void publish(const ros::Time& currentTime);
	
	static void publishAgents(ros::Publisher& agentsPub, const ros::Time& currentTime);
	static void publishNodes(ros::Publisher& nodesPub);
	static void publishEdges(ros::Publisher& edgesPub);
	static void publishForcesVisualization(ros::Publisher& forcesPub,const ros::Time& currentTime);
	static void publishNodesVisualization(ros::Publisher& nodesPub, const ros::Time& currentTime);
	static void publishEdgesVisualization(ros::Publisher& edgesPub, const ros::Time& currentTime);
	static void publishAgentsVisualization(ros::Publisher& agentsPub, const ros::Time& currentTime);

private:
	static void agentToMsg(const Agent& agent, tars::AgentMsg& msg);
	static std_msgs::ColorRGBA getColor(double r, double g, double b, double a);
	static void publishForceMarker(const ros::Time& currentTime,
					unsigned index, 
					const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, 
					const utils::Vector2d& position,
					visualization_msgs::MarkerArray& markers);
	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	Robot* robot;
	ros::Publisher odomPublisher;
	ros::Publisher scanPublisher;
	ros::Publisher trackingVisPub;
	ros::Publisher trackingPub;
	ros::Subscriber cmdVelSubscriber;
	sensor_msgs::LaserScan scan;
	utils::Vector2d lastScanPosition;
	utils::Angle lastScanYaw;
};




inline
void ROSInterface::publish(const ros::Time& currentTime) {
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = currentTime;
	transform.header.frame_id = "map";
	transform.child_frame_id = robot->getName() + "/odom";
	transform.transform.translation.x = robot->getInitialPosition().getX();
	transform.transform.translation.y = robot->getInitialPosition().getY();
	transform.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, robot->getInitialYaw().toRadian());
	transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
	br.sendTransform(transform);
	double x = robot->getPosition().getX() - robot->getInitialPosition().getX();
	double y = robot->getPosition().getY() - robot->getInitialPosition().getY();
	double a = -robot->getInitialYaw().toRadian();
	double c = std::cos(a);
	double s = std::sin(a);
	transform.header.frame_id = robot->getName() + "/odom";
	transform.child_frame_id = robot->getName() + "/base_link";
	transform.transform.translation.x = x*c - y*s;
	transform.transform.translation.y = x*s + y*c; 
	q.setRPY(0, 0, (robot->getYaw() - robot->getInitialYaw()).toRadian());
	transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
	br.sendTransform(transform);
	nav_msgs::Odometry odom;
	odom.header.stamp = currentTime;
	odom.header.frame_id = robot->getName() + "/odom";
	odom.child_frame_id =  robot->getName() + "/base_link";
	odom.pose.pose.position.x = transform.transform.translation.x;
	odom.pose.pose.position.y = transform.transform.translation.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = transform.transform.rotation;
	odom.twist.twist.linear.x = robot->getLinVel();
	odom.twist.twist.linear.y = 0.0; 
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = robot->getAngVel();
	odomPublisher.publish(odom);
	scan.header.stamp = currentTime;
	if (lastScanPosition != robot->getPosition() || lastScanYaw != robot->getYaw()) {	
		lastScanPosition = robot->getPosition();
		lastScanYaw =robot->getYaw();
		scan.ranges = robot->getScan();
	}
	scanPublisher.publish(scan);
	tars::AgentsMsg msg;
	msg.header.frame_id = "map";
	msg.header.stamp = currentTime;
	msg.size = robot->getAgentsDetected().size();
	msg.agents.resize(msg.size);
	std::unordered_set<std::string> names;
	for (unsigned i=0;i<msg.size;i++) {
		agentToMsg(*AGENTS[robot->getAgentsDetected()[i]],msg.agents[i]);
		if (AGENTS[robot->getAgentsDetected()[i]]->getName() != robot->getName()) {
			names.insert(AGENTS[robot->getAgentsDetected()[i]]->getName());
		}
	}
	trackingPub.publish(msg);
	visualization_msgs::MarkerArray markers;
	for (unsigned i=0;i<AGENTS.size();i++) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
	    marker.header.stamp = currentTime;
	    if (names.count(AGENTS[i]->getName())>0) {
			marker.action = visualization_msgs::Marker::ADD;	
		} else {
			marker.action = visualization_msgs::Marker::DELETE;

		}
		marker.color.a = 0.5;	
		marker.id = i;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.color.r = 1.0;
	    marker.color.g = 0;
	    marker.color.b = 0;
	    marker.scale.x = AGENTS[i]->getRadius()*2;
		marker.scale.y = AGENTS[i]->getRadius()*2;
		if (AGENTS[i]->getType()==HUMAN) {
			marker.scale.z = 1.2;
		} else {
			marker.scale.z = 0.3;
		}
		marker.pose.position.x = AGENTS[i]->getPosition().getX();
		marker.pose.position.y = AGENTS[i]->getPosition().getY();
		marker.pose.position.z = marker.scale.z / 2;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;		
		marker.pose.orientation.z = 0;		
		marker.pose.orientation.w = 1.0;			
	    markers.markers.push_back(marker);
	}
	trackingVisPub.publish(markers);
}






*/

#endif