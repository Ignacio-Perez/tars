/*
MIT License

Copyright (c) 2021 Ignacio Pérez Hurtado de Mendoza

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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tars/TARS.hpp>
#include <tars/ros_interface.hpp>
#include <tars/AgentsMsg.h>
#include <tars/RobotGoalSrv.h>


bool setRobotGoal(tars::RobotGoalSrv::Request  &req,
             tars::RobotGoalSrv::Response &res) {
	
	for (unsigned i=0;i<AGENTS.size();i++) {
		if (AGENTS[i]->getName()==req.id && AGENTS[i]->getType()==ROBOT) {
			Robot* robot = (Robot*)AGENTS[i];
			utils::Vector2d goal(req.gx,req.gy);
			robot->setGoal(goal);
			res.error=false;
			return true;
		}
	}
	res.error=true;
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"TARS_node");
	ros::NodeHandle n, pn("~");
	std::string scenario;
	pn.param<std::string>("scenario",scenario,"");
	try{
		ROS_INFO("Loading scenario '%s'",scenario.c_str());
		TARS.load(scenario);
		ROS_INFO("Done!");
	} catch (const std::exception& e) {
		ROS_FATAL("%s",e.what());
		return 1;
	}
	std::vector<ROSInterface> interfaces;
	for (unsigned i=0; i< AGENTS.size();i++) {
		if (AGENTS[i]->getType()==ROBOT) {
			interfaces.emplace_back((Robot*)AGENTS[i],n,pn);
		}
	}
	ros::Publisher nodesPub = pn.advertise<geometry_msgs::Polygon>("/tars/nodes",1,true);
	ros::Publisher edgesPub = pn.advertise<geometry_msgs::Polygon>("/tars/edges",1,true);
	ros::Publisher agentsVisPub = pn.advertise<visualization_msgs::MarkerArray>("/tars/visualization/agents",1);
	ros::Publisher nodesVisPub = pn.advertise<visualization_msgs::MarkerArray>("/tars/visualization/nodes",1);
	ros::Publisher edgesVisPub = pn.advertise<visualization_msgs::MarkerArray>("/tars/visualization/edges",1);
	ros::Publisher forcesVisPub = pn.advertise<visualization_msgs::MarkerArray>("/tars/visualization/forces",1);
	ros::Publisher agentsPub = pn.advertise<tars::AgentsMsg>("/tars/agents",1);
	ros::ServiceServer service = n.advertiseService("/tars/robot_goal", setRobotGoal);
	ros::Rate r(TARS.getFreq());	
	ros::Time currentTime = ros::Time::now();	
 	ros::Time prevTime = currentTime;
 	bool graphPublished=false;
	while (n.ok()) {
		TARS.update((currentTime - prevTime).toSec());
		for (auto it = interfaces.begin(); it!= interfaces.end(); ++it) {
			it->publish(currentTime);
		}
		if (!graphPublished) {
			ROSInterface::publishNodes(nodesPub);
			ROSInterface::publishEdges(edgesPub);
			graphPublished=true;
		}
		ROSInterface::publishAgents(agentsPub,currentTime);
		ROSInterface::publishNodesVisualization(nodesVisPub,currentTime);
		ROSInterface::publishEdgesVisualization(edgesVisPub,currentTime);
		ROSInterface::publishForcesVisualization(forcesVisPub,currentTime);
		ROSInterface::publishAgentsVisualization(agentsVisPub,currentTime);
		r.sleep();
		ros::spinOnce();
		prevTime = currentTime;
		currentTime = ros::Time::now();	
	}
	return 0;
}