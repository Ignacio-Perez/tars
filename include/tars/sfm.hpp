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

#ifndef _SFM_HPP_
#define _SFM_HPP_

#include <string>
#include <vector>
#include <list>
#include <limits>
#include <map>
#include "vector2d.hpp"
#include "substr.hpp"
#include "random.hpp"
#include "timer.hpp"
#include "graph.hpp"

struct Forces
{
	utils::Vector2d desiredForce;
	utils::Vector2d obstacleForce;
	utils::Vector2d socialForce;
	utils::Vector2d globalForce;
};

struct Parameters
{
	Parameters()
	: forceFactorDesired(0.8), 
	  forceFactorObstacle(3.6), 
	  forceSigmaObstacle(0.2),
	  forceFactorSocial(2.2),
	  obstacleReadings(8),  
	  lambda(2.0),
	  gamma(0.35),
	  n1(2.0),
	  n2(3.0),
	  relaxationTime(0.5) {}
	double forceFactorDesired;
	double forceFactorObstacle;
	double forceSigmaObstacle;
	double forceFactorSocial;
	unsigned obstacleReadings;
	double lambda;
	double gamma;
	double n1;
	double n2;
	double relaxationTime;
};

enum Type {ROBOT, HUMAN};

class Agent
{
public:
	Agent() : forcesEnabled(false), radius(0.2), desiredVelocity(0.6) {}
	virtual ~Agent() {}
	const std::string& getName() const {return name;}
	const utils::Vector2d& getPosition() const {return position;}
	const utils::Vector2d& getVelocity() const {return velocity;}
	const utils::Angle& getYaw() const {return yaw;}
	double getRadius() const {return radius;}
	const Forces& getForces() const {return forces;}
	const Parameters& getParameters() const {return params;}
	void setParameters(const Parameters& params) {Agent::params = params;}
	bool hasForces() const {return forcesEnabled;}
	double getDesiredVelocity() const {return desiredVelocity;}
	void computeForces(const Map& map,const std::vector<Agent*>& agents);
	bool checkCollision(const Map& map,const std::vector<Agent*>& agents) const;
	virtual void update(double dt,const Map& map,const std::vector<Agent*>& agents) = 0;
	virtual Type getType() const = 0;
	virtual const utils::Vector2d& getGoal() const = 0;
	
protected:	
	#define PW(x) ((x)*(x))
	void computeDesiredForce();
	void computeObstacleForce(const Map& map);
	void computeSocialForce(const std::vector<Agent*>& agents,const Map& map);

	bool forcesEnabled;
	Forces forces;
	Parameters params;
	double radius;
	double desiredVelocity;
	utils::Vector2d position;
	utils::Vector2d velocity;
	utils::Angle yaw;
	std::string name;
};

inline
bool Agent::checkCollision(const Map& map,const std::vector<Agent*>& agents) const {
	if (map.getDistanceToNearestObstacle(position) < radius) {
		return true;
	}
	for (unsigned i=0;i<agents.size();i++) {
		if (agents[i] == this) {
			continue;
		}
		if ( (position-agents[i]->getPosition()).norm() < radius + agents[i]->getRadius()) {
			return true;
		}
	}
	return false;
}


class Human : public Agent
{
public:
	Human(const std::string& yaml, Graph& graph, Map& map);
	~Human() {}
	void update(double dt, const Map& map,const std::vector<Agent*>& agents);
	Type getType() const {return HUMAN;}
	void updatePath(); 
	const utils::Vector2d& getGoal() const {
		return path.empty() ? graph->getNodes().at(goal) : graph->getNodes().at(path.front());
	}
	
private:
	std::vector<std::string> goals;
	std::list<std::string> path;
	std::vector<Edge> edges;
	Timer timer0;
	Timer timer1;
	double timeInGoal;
	Graph* graph;
	Map* map;
	std::string goal;
};

class Robot : public Agent
{
public:
	Robot(const std::string& yaml);
	~Robot() {}
	Type getType() const {return ROBOT;}
	const utils::Vector2d& getGoal() const {return goal;}
	void setGoal(const utils::Vector2d& goal) {Robot::goal = goal;forcesEnabled=true;}
	void setDesiredVelocity(double desiredVelocity) {Agent::desiredVelocity = desiredVelocity;}
	void enableForces(bool forcesEnabled) {Agent::forcesEnabled = forcesEnabled;}
	const utils::Vector2d& getInitialPosition() const {return initialPosition;}
	const utils::Angle& getInitialYaw() const {return initialYaw;}
	double getLinVel() const {return linVel;}
	double getAngVel() const {return angVel;}
	void setCmdVel(double linVel, double angVel);
	unsigned getScanReadings() const {return scan.size();}
	double getScanRange() const {return scanRange;}
	const std::vector<float>& getScan() const {return scan;}
	double getScanAngleIncrement() const {return scanAngleIncrement;}
	double getAgentsDetectionRange() const {return agentsDetectionRange;}
	void updateSensors(const Map& map,const std::vector<Agent*>& agents);
	void update(double dt, const Map& map, const std::vector<Agent*>& agents);
	const std::vector<unsigned>& getAgentsDetected() const {return agentsDetected;}
private:
	utils::Vector2d goal;
	double linVel;
	double angVel;	
	utils::Vector2d initialPosition;
	utils::Angle initialYaw;
	double scanRange;
	double scanAngleIncrement;
	double agentsDetectionRange;
	utils::Vector2d lastScanPosition;
	utils::Angle lastScanYaw;
	std::vector<float> scan;
	std::vector<unsigned> agentsDetected;
};


inline
void Robot::updateSensors(const Map& map, const std::vector<Agent*>& agents) {
	
	if (lastScanPosition != position || lastScanYaw != yaw) {	
		
		lastScanPosition = position;
		lastScanYaw = yaw;
		double angle = yaw.toRadian() - M_PI;
		utils::Vector2d u,p;
		double d;
		double obstacleRadius = map.getResolution()/2;
		
		for (unsigned i=0;i<scan.size();i++) {
			u.set(std::cos(angle), std::sin(angle));
			p = position + u*radius;
			u *= obstacleRadius;
			d = radius;
			while (d <= scanRange && !map.isObstacle(p)) {
				p += u;
				d += obstacleRadius;
			}
			scan[i] = std::max(radius,std::min(d,scanRange));
			angle += scanAngleIncrement;
		}	
	}
	
	agentsDetected.clear();
	for (unsigned i = 0; i< agents.size();i++) {
		if ((position - agents[i]->getPosition()).norm() < agentsDetectionRange &&
			map.isReachableInStraightLine(position,agents[i]->getPosition() )) {
			agentsDetected.push_back(i);
		}
	}
	
}

inline
Robot::Robot(const std::string& yaml) 
: linVel(0), angVel(0) {
	unsigned pos = 0;
	unsigned length = substr(yaml,'\"','\"',pos);
	name = yaml.substr(pos,length);
	length = substr(yaml,',',',',pos);
	desiredVelocity = std::stod(yaml.substr(pos,length));
	length = substr(yaml,',',',',pos);
	radius = std::stod(yaml.substr(pos,length));
	length = substr(yaml,',',',',pos);
	unsigned scanReadings = std::stoi(yaml.substr(pos,length));
	length = substr(yaml,',',',',pos);
	scanRange = std::stod(yaml.substr(pos,length));
	length = substr(yaml,',',',',pos);
	agentsDetectionRange = std::stod(yaml.substr(pos,length));
	length = substr(yaml,',',',',pos);
	std::string x = yaml.substr(pos,length);
	length = substr(yaml,',',',',pos);
	std::string y = yaml.substr(pos,length);
	length = substr(yaml,',',']',pos);
	std::string a = yaml.substr(pos,length);
	position.set(std::stod(x),std::stod(y));
	yaw.setRadian(stod(a));
	scan.resize(scanReadings);
	scanAngleIncrement = 2*M_PI/(double)scanReadings;
	initialPosition = position;
	initialYaw = yaw;
}

inline
void Robot::setCmdVel(double linVel, double angVel) {
	Robot::linVel = linVel;
	Robot::angVel = angVel;
}

inline
void Robot::update(double dt, const Map& map,const std::vector<Agent*>& agents) {
	if (linVel!=0 || angVel!=0) {
		double imd = linVel * dt;
		utils::Vector2d inc(imd * std::cos(yaw.toRadian() + angVel*dt*0.5), imd * std::sin(yaw.toRadian() + angVel*dt*0.5));
		position += inc;
		if (checkCollision(map,agents)) {
			position -= inc;
			velocity.set(0,0);
			linVel = 0;
			angVel = 0;
		} else {
			yaw += utils::Angle::fromRadian(angVel * dt);	
			velocity.set(linVel * yaw.cos(), linVel * yaw.sin());
		}
	} else {
		velocity.set(0,0);
	}
}

inline
void Human::update(double dt, const Map& ,const std::vector<Agent*>& ) {
	velocity += forces.globalForce * dt;
	if (velocity.norm() > desiredVelocity) {
		velocity.normalize();
		velocity *= desiredVelocity;
	}
	position += velocity * dt;
	yaw = velocity.angle();
}


inline
Human::Human(const std::string& yaml, Graph& graph, Map& map)
: graph(&graph), map(&map) {
	unsigned pos = 0;
	unsigned length = substr(yaml,'\"','\"',pos);
	name = yaml.substr(pos,length);
	length = substr(yaml,',',',',pos);
	desiredVelocity = std::stod(yaml.substr(pos,length));
	while(yaml.find_first_of('\"',pos)!= std::string::npos) {
		length = substr(yaml,'\"','\"',pos);
		goals.push_back(yaml.substr(pos,length));
		pos+=length+1;
	}
	std::vector<std::string> aux;
	for (auto it = goals.begin();it!=goals.end();++it) {
		if (!graph.isSelected(*it)) {
			aux.push_back(*it);
		}
	}
	forcesEnabled = true;
	goal = aux[RANDOM(aux.size())];
	position = graph[goal];
	yaw = graph.getAngle(goal);
	graph.select(goal);
	timer0.reset();
	timeInGoal = RANDOM(10,1);
}

inline
void Human::updatePath() {
	if (path.empty() && timer0.elapsed()>timeInGoal) {
		std::vector<std::string> aux;
		for (unsigned k=0;k<goals.size();k++) {
			if (!graph->isSelected(goals[k])) {
				aux.push_back(goals[k]);
			}
		}
		if (!aux.empty()) {
			std::string current = goal;
			goal = aux[RANDOM(aux.size())];
			graph->unselect(current);
			graph->select(goal);
			if (!graph->aStar(current,goal,path)) {
				graph->unselect(goal);
				graph->select(current);
				goal = current;
			}
		}
	} 
	while (!path.empty() && (position-graph->getNodes().at(path.front())).norm()<2*radius) {
		timer1.reset();
		for (unsigned i=0;i<edges.size();i++) {
			graph->dec(edges[i]);
		}
		std::string current = path.front();
		path.pop_front();
		if (path.empty()) {
			edges = graph->getNeighbourhood(goal);
			timer0.reset();
			timeInGoal = RANDOM(60,15);
		} else {
			graph->aStar(current,goal,path);
			path.pop_front();
			edges.clear();
			edges.push_back(graph->getEdge(current,path.front()));
		}
		for (unsigned i=0;i<edges.size();i++) {
			graph->inc(edges[i]);
		}
	}

	if (!path.empty() && timer1.elapsed()>60) {
		std::string newNode;
		double minDist = std::numeric_limits<double>::infinity();
		for (auto it = graph->getNodes().begin();it!= graph->getNodes().end();++it) {
			if (map->isReachableInStraightLine(position,it->second) && it->first != path.front() && it->first!=goal) {
				double dist = (position - it->second).norm();
				if (dist < minDist) {
					minDist = dist;
					newNode = it->first;
				}
			}
		}
		if (minDist < std::numeric_limits<double>::infinity()) {
			for (unsigned i=0;i<edges.size();i++) {
				graph->dec(edges[i]);
			}
			edges.clear();
			path.clear();
			if (!graph->aStar(newNode,goal,path)) {
				graph->unselect(goal);
				goal = newNode;
				path.push_back(newNode);
			}
		} 
	}
}

inline
void Agent::computeDesiredForce() {
	if ((position- getGoal()).norm()>2*radius) {
		utils::Vector2d diff = getGoal() - position;
		utils::Vector2d desiredDirection = diff.normalized();
		forces.desiredForce = params.forceFactorDesired * (desiredDirection * desiredVelocity  - velocity)/params.relaxationTime;
	} else {
		forces.desiredForce = -velocity / params.relaxationTime;
		
	}
}

inline
void Agent::computeSocialForce(const std::vector<Agent*>& agents, const Map& map) 
{
	forces.socialForce.set(0,0);
	for (unsigned i = 0; i< agents.size(); i++) {
		if (agents[i] == this || !map.isReachableInStraightLine(position,agents[i]->getPosition())) {
			continue;
		}
		utils::Vector2d diff = agents[i]->getPosition() - position;
		utils::Vector2d diffDirection = diff.normalized();
		utils::Vector2d velDiff = velocity - agents[i]->getVelocity();
		utils::Vector2d interactionVector = params.lambda * velDiff + diffDirection;
		double interactionLength = interactionVector.norm();
		utils::Vector2d interactionDirection = interactionVector/interactionLength;
		utils::Angle theta = interactionDirection.angleTo(diffDirection);
		double B = params.gamma * interactionLength;
		double thetaRad = theta.toRadian();	
		double forceVelocityAmount = -std::exp(-diff.norm()/B - PW(params.n2*B*thetaRad) );
		double forceAngleAmount = -theta.sign() * std::exp(-diff.norm() / B - PW(params.n1 * B * thetaRad));
		utils::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
		utils::Vector2d forceAngle = forceAngleAmount * interactionDirection.leftNormalVector();
		forces.socialForce += params.forceFactorSocial * (forceVelocity + forceAngle);
	}
}

inline
void Agent::computeObstacleForce(const Map& map) 
{
	forces.obstacleForce.set(0,0);
	utils::Angle a;
	utils::Angle offset = utils::Angle::fromDegree(360.0 / (double)params.obstacleReadings);
	double factor = params.forceFactorObstacle / (double) params.obstacleReadings;
	for (unsigned i=0;i<params.obstacleReadings;i++) {
		utils::Vector2d u(a.cos(),a.sin());
		u*=radius;
		utils::Vector2d p = position+u;
		utils::Vector2d o = map.getNearestObstacle(p);
		utils::Vector2d minDiff = position - o;
		double distance = minDiff.norm() - radius;
		forces.obstacleForce += factor * std::exp(-distance/params.forceSigmaObstacle) * minDiff.normalized();
		a+=offset;
	}
}

inline
void Agent::computeForces(const Map& map,const std::vector<Agent*>& agents) {
	computeDesiredForce();
	computeObstacleForce(map);
	computeSocialForce(agents,map);
	forces.globalForce = forces.desiredForce + forces.socialForce + forces.obstacleForce;		
}

#endif
