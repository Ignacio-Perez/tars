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

#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <string>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <forward_list>
#include <limits>
#include "map.hpp"
#include "angle.hpp"
#include "substr.hpp"

class Edge
{
public:
	Edge(const std::string& src, const std::string& dst, double distance)
	: src(src), dst(dst), distance(distance) {}
	const std::string& getSrc() const {return src;}
	const std::string& getDst() const {return dst;}
	const std::string& getComp(const std::string& node) const {return node == src ? dst : src;}
	double getDistance() const {return distance;}
	bool operator==(const Edge& other) const {return src == other.src && dst == other.dst;}
	bool operator<(const Edge& other) const {return src < other.src || (src == other.src && dst < other.dst);}
	friend class Graph;
private:
	std::string src;
	std::string dst;
	double distance;
};

class Graph
{
public:
	const std::map<std::string, utils::Vector2d>& getNodes() const {return nodes;}
	const std::map<Edge,double>& getEdges() const {return edges;}
	const utils::Vector2d& operator[](const std::string& nodeId) const {return nodes.at(nodeId);}
	const std::vector<Edge>& getNeighbourhood(const std::string& nodeId) const {return neighbourhoods.at(nodeId);}
	void inc(const Edge& edge) {edges.at(edge) += edge.getDistance();}
	void dec(const Edge& edge) {edges.at(edge) -= edge.getDistance();}
	double getCost(const Edge& edge) const {return edges.at(edge);}
	bool isSelected(const std::string& goalId) const {return goalsSelected.count(goalId)>0;}
	void select(const std::string& goalId) {goalsSelected.insert(goalId);}
	void unselect(const std::string& goalId) {goalsSelected.erase(goalId);}
	const utils::Angle& getAngle(const std::string& goalId) const {return angles.at(goalId);}
	void addNode(const std::string& yaml);
	void addGoal(const std::string& yaml);
	void createEdges(const Map& map);
	const Edge& getEdge(const std::string& src, const std::string& dst) const;
	double getMinCost() const;
	double getMaxCost() const;
	double aStar(const std::string& start, const std::string& goal, std::list<std::string>& path) const;

private:
	struct Info {
		void set(double gScore,  double fScore) {
			Info::gScore = gScore;
			Info::fScore = fScore;
		}
		double gScore;
		double fScore;
	};
	class PriorityQueue
	{
	public:
		unsigned empty() const {
			return nodes.empty();
		}
		const std::string& top() const {
			return *(queue.begin()->second.begin());
		}
		void pop() {
			nodes.erase(top());
			queue.begin()->second.erase(queue.begin()->second.begin());
			if (queue.begin()->second.empty()) {
				queue.erase(queue.begin());
			}
		}
		void push(const std::string& node, double gScore, double fScore) {
			if (nodes.count(node)==0) {
				nodes.insert(node);
			} else {
				double fs = info.at(node).fScore;
				queue.at(fs).erase(node);
				if (queue.at(fs).empty()) {
					queue.erase(fs);
				}
			}
			info[node].set(gScore,fScore);
			queue[fScore].insert(node);
		}
		double operator[](const std::string& node) const {
			return info.count(node)==0 ? std::numeric_limits<double>::infinity() : info.at(node).gScore;
		}

	private:
		std::map<double,std::set<std::string>> queue;
		std::unordered_set<std::string> nodes;
		std::unordered_map<std::string,Info> info;
	};
	std::map<std::string, utils::Vector2d> nodes;
	std::map<Edge,double> edges;
	std::unordered_map<std::string, utils::Angle> angles;
	std::unordered_set<std::string> goalsSelected;
	std::unordered_map<std::string, std::vector<Edge> > neighbourhoods;	
};

inline
void Graph::addNode(const std::string& yaml) {
	unsigned pos = 0;
	unsigned length = substr(yaml,'\"','\"',pos);
	std::string name = yaml.substr(pos,length);
	length = substr(yaml,',',',',pos);
	std::string x = yaml.substr(pos,length);
	length = substr(yaml,',',']',pos);
	std::string y = yaml.substr(pos,length);
	nodes[name].set(std::stod(x),std::stod(y));
}

inline
void Graph::addGoal(const std::string& yaml) {
	unsigned pos = 0;
	unsigned length = substr(yaml,'\"','\"',pos);
	std::string name = yaml.substr(pos,length);
	length = substr(yaml,',',',',pos);
	std::string x = yaml.substr(pos,length);
	length = substr(yaml,',',',',pos);
	std::string y = yaml.substr(pos,length);
	nodes[name].set(std::stod(x),std::stod(y));
	length = substr(yaml,',',']',pos);
	std::string a = yaml.substr(pos,length);
	angles[name].setRadian(std::stod(a));
}

inline
void Graph::createEdges(const Map& map) {
	for (auto it1 = nodes.begin();it1!=nodes.end();++it1) {
		for (auto it2 = nodes.begin();it2!=nodes.end();++it2) {
			if (it1->first == it2->first || !map.isReachableInStraightLine(it1->second,it2->second)) {
				continue;
			}
			Edge e(it1->first, it2->first, (it1->second - it2->second).norm());
			if (it2->second < it1->second) {
				std::swap(e.src, e.dst);
			}
			if (edges.count(e) == 0) {
				edges[e] = e.getDistance();
				neighbourhoods[it1->first].push_back(e);
				neighbourhoods[it2->first].push_back(e);
			}
		}
	}
	auto it = edges.begin();
	std::list<std::string> path;
	std::forward_list<int> l;
	while (it!=edges.end()) {
		double d = it->second;
		it->second += d;
		double cost = aStar(it->first.getSrc(),it->first.getDst(),path);
		if (std::abs(cost - d) < 1e-7) {
			it = edges.erase(it);
			neighbourhoods.clear();
			for (auto it1 = edges.begin(); it1!= edges.end(); ++it1) {
				neighbourhoods[it1->first.getSrc()].push_back(it1->first);
				neighbourhoods[it1->first.getDst()].push_back(it1->first);
			}
		} else {
			l.push_front((int)std::floor(d/0.05));
			it->second -= d;
			++it;
		}
	}
}

inline
const Edge& Graph::getEdge(const std::string& src, const std::string& dst) const {
	Edge e(src,dst,0);
	if (edges.count(e)==0) {
		std::swap(e.src, e.dst);
	}
	return edges.find(e)->first;
}

inline
double Graph::getMinCost() const {
	auto it = edges.begin();
	double min = it->second;
	++it;
	while (it!= edges.end()) {
		if (it->second < min) {
			min = it->second;
		}
		++it;
	}
	return min;
}

inline
double Graph::getMaxCost() const {
	auto it = edges.begin();
	double max = it->second;
	++it;
	while (it!= edges.end()) {
		if (it->second > max) {
			max = it->second;
		}
		++it;
	}
	return max;
}

inline
double Graph::aStar(const std::string& start, const std::string& goal, std::list<std::string>& path) const {
	PriorityQueue openSet;
	std::unordered_map<std::string,std::string> cameFrom;
	openSet.push(start,0,(nodes.at(start) - nodes.at(goal)).norm());
	while (!openSet.empty()) {
		std::string current = openSet.top();
		if (current == goal) {
			double cost = 0;
			path.clear();
			path.push_front(current);
			while (cameFrom.count(current)!=0) {
				std::string aux = cameFrom.at(current);
				cost += edges.at(getEdge(aux,current));
				current = aux;
				path.push_front(current);
			}
			return cost;
		}
		openSet.pop();
		double gScore = openSet[current];
		const std::vector<Edge>& neighbourhood = neighbourhoods.at(current);
		for (unsigned i=0;i< neighbourhood.size();i++) {
			const Edge& edge = neighbourhood[i];
			const std::string& neighbour = edge.getComp(current);
			double tentative_gScore = gScore + edges.at(edge);
			if (tentative_gScore < openSet[neighbour]) {
				cameFrom[neighbour] = current;
				openSet.push(neighbour,tentative_gScore, tentative_gScore + (nodes.at(neighbour) - nodes.at(goal)).norm());
			}
		} 
	}
	return std::numeric_limits<double>::infinity();
}

#endif