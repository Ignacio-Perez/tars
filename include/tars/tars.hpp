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

#ifndef _TARS_HPP_
#define _TARS_HPP_

#include <fstream>
#include <string>
#include <map>
#include "map.hpp"
#include "sfm.hpp"
#include "graph.hpp"

class Tars
{
public:
	~Tars();
	Tars(const Tars& other) = delete;
	void operator=(const Tars& other) = delete;
	static Tars& getInstance() {static Tars instance;return instance;}
	#define TARS Tars::getInstance()
	#define AGENTS Tars::getInstance().getAgents()
	#define GRAPH Tars::getInstance().getGraph()
	#define MAP Tars::getInstance().getMap()
	void load(const std::string& fileName);
	const std::vector<Agent*> getAgents() const {return agents;}
	const Graph& getGraph() const {return graph;}
	const Map& getMap() const {return map;}
	double getFreq() const {return freq;}
	const Parameters& getParams() const {return params;}
	void update(double dt);

private:
	Tars() : freq(10.0) {}	
	void addGoal(const std::string& yaml);
	static double getDouble(const std::string& yaml);
	static int getInt(const std::string& yaml);
	double freq;
	Parameters params;
	Map map;
	Graph graph;
	std::vector<Agent*> agents;
};

inline
Tars::~Tars() {
	for (unsigned i=0;i<agents.size();i++) {
		delete agents[i];
	}
}

inline 
void Tars::update(double dt) {
	for (unsigned i=0;i<agents.size();i++) {
		if (agents[i]->getType()==HUMAN) {
			Human* human = (Human*)agents[i];
			human->updatePath();
		} 
		if (agents[i]->hasForces()) {
			agents[i]->computeForces(map,agents);
		}
	}
	for (unsigned i=0;i<agents.size();i++) {
		agents[i]->update(dt,map,agents);
	}
	for (unsigned i=0;i<agents.size();i++) {
		if (agents[i]->getType()==ROBOT) {
			Robot* robot = (Robot*)agents[i];
			robot->updateSensors(map,agents);
		} 
	}
}

inline
double Tars::getDouble(const std::string& yaml) {
	unsigned pos = yaml.find_first_of(':');
	if (pos == std::string::npos) {
		throw std::runtime_error("Invalid syntax: "+yaml);
	}
	++pos;
	while(pos < yaml.size() && std::isspace(yaml[pos])) {
		++pos;
	}
	if (pos == yaml.size()) {
		throw std::runtime_error("Invalid syntax: "+yaml);
	}
	return std::stod(yaml.substr(pos));
}

inline
int Tars::getInt(const std::string& yaml) {
	unsigned pos = yaml.find_first_of(':');
	if (pos == std::string::npos) {
		throw std::runtime_error("Invalid syntax: "+yaml);
	}
	++pos;
	while(pos < yaml.size() && std::isspace(yaml[pos])) {
		++pos;
	}
	if (pos == yaml.size()) {
		throw std::runtime_error("Invalid syntax: "+yaml);
	}
	return std::stoi(yaml.substr(pos));
}


inline
void Tars::load(const std::string& fileName) {
	std::fstream file(fileName,std::ios::in);
	if (!file.is_open()) {
		throw std::runtime_error("Cannot open scenario file '" + fileName+"'");
	}
	unsigned pos = fileName.find_last_of('/');
	std::string workingDirectory = pos == std::string::npos ? "" : fileName.substr(0,pos);
	int counter=1;
	std::string line;
	bool firstHuman=true;
	while (std::getline(file,line)) {
		if (line.find("freq:")==0) {
			freq = getDouble(line);
		} else if (line.find("forceFactorDesired:")==0) {
			params.forceFactorDesired = getDouble(line);
		} else if (line.find("forceFactorObstacle:")==0) {
			params.forceFactorObstacle = getDouble(line);
		} else if (line.find("forceFactorSocial:")==0) {
			params.forceFactorSocial = getDouble(line);
		} else if (line.find("obstacleReadings:")==0) {
			params.obstacleReadings = getInt(line);
		} else if (line.find("forceSigmaObstacle:")==0) {
			params.forceSigmaObstacle = getDouble(line);
		} else if (line.find("lambda:")==0) {
			params.lambda = getDouble(line);
		} else if (line.find("gamma:")==0) {
			params.gamma = getDouble(line);
		} else if (line.find("n1:")==0) {
			params.n1 = getDouble(line);
		} else if (line.find("n2:")==0) {
			params.n2 = getDouble(line);
		} else if (line.find("relaxationTime:")==0) {
			params.relaxationTime = getDouble(line);
		} else if (line.find("map:")==0) {
			map.load(workingDirectory,line);
		} else if (line.find("robot:")==0) {
			agents.push_back(new Robot(line));
		} else if (line.find("human:")==0) {
			if (firstHuman) {
				graph.createEdges(map);
				firstHuman=false;
			}
			agents.push_back(new Human(line,graph,map));
		} else if (line.find("node:")==0) {
			graph.addNode(line);
		} else if (line.find("goal:")==0) {
			graph.addGoal(line);
		} else if (!line.empty() && line.find("#") !=0) {
			throw std::runtime_error("Invalid syntax in scenario file (line "+std::to_string(counter)+"): "+line);
		}
		++counter;
	}
	file.close();
	if (!map.isLoaded()) {
		throw std::runtime_error("No map loaded");
	}
	for (unsigned i=0;i<agents.size();i++) {
		agents[i]->setParameters(params);
	}
}

#endif
