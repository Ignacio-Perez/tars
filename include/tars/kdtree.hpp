#pragma once


#include <algorithm>
#include <vector>
#include <limits>
#include "vector2d.hpp"

namespace utils
{

class KdTree
{
public:

    KdTree() {}

    KdTree(const utils::Vector2d& pos) {
        nodes.emplace_back(pos);
    }

    KdTree(std::vector<utils::Vector2d>& points)  {
        createKdTree(points,0,points.size(),true);
    }

    void clear() {
        nodes.clear();
    }
    
    unsigned size() const {
        return nodes.size();
    }

    const utils::Vector2d& operator[](unsigned index) const {
        return nodes[index].location;
    }

    const utils::Vector2d& getNearestPoint(const utils::Vector2d& point) const {
        return nodes[getNearest(point)].location;
    }

    double getDistanceToNearestPoint(const utils::Vector2d& point) const {
        if (nodes.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        double distance = 0;
        searchNearest(0,point,distance,true);
        return distance;
    }

    unsigned getNearest(const utils::Vector2d& point) const {
        double distance=0;
        return searchNearest(0,point,distance,true);
    }

    void getKNearests(const utils::Vector2d& point, std::vector<unsigned>& result, unsigned k) const {
        std::vector<Point> queue;
        searchNearest(0, point, true, queue, k);
        result.clear();
        for (unsigned i=0;i<queue.size();i++) {
            result.push_back(queue[i].index);
        }
    }

    void add(double x, double y) {
        utils::Vector2d p(x,y);
        add(p);
    }

    void add (const utils::Vector2d& point) {
        nodes.emplace_back(point);
        if (nodes.size()==1) {
            return;
        }

        int index = 0;
        bool axis = true;
        while (true) {
            if ( (axis && lessX(point,nodes[index].location)) || (!axis && lessY(point,nodes[index].location))) {
                if (nodes[index].left==-1) {
                    nodes[index].left = nodes.size()-1;
                    break;
                }
                index = nodes[index].left;
            } else {
                if (nodes[index].right==-1) {
                    nodes[index].right = nodes.size()-1;
                    break;
                }
                index = nodes[index].right;
            }
            axis = !axis;
        }
    }

private:
    struct Node {
        Node(const utils::Vector2d& location) : location(location), left(-1), right(-1) {}
        utils::Vector2d location;
        int left;
        int right;
    };

    struct Point {
        Point() {}
        Point(unsigned index, double distance) : index(index), distance(distance) {}
        unsigned index;
        double distance;
        bool operator<(const Point& other) const {
            return distance < other.distance;
        }
    };

    static bool lessX(const utils::Vector2d& a, const utils::Vector2d& b) {
        return a.getX() < b.getX();
    }
    static bool lessY(const utils::Vector2d& a, const utils::Vector2d& b) {
        return a.getY() < b.getY();
    }

    int createKdTree(std::vector<utils::Vector2d>& points, int begin, int end, bool axis) {
        if (end <= begin) {
            return -1;
        }
        if (axis) {
            std::sort(points.begin()+begin, points.begin()+end, lessX);
        } else {
            std::sort(points.begin()+begin, points.begin()+end, lessY);
        }
        int median = begin + (end-begin)/2;
        int index = (int)nodes.size();
        nodes.emplace_back(points[median]);
        nodes[index].left = createKdTree(points,begin,median,!axis);
        nodes[index].right = createKdTree(points,median+1,end,!axis);
        return index;
    }


    void searchNearest(int index, const utils::Vector2d& point, 
         bool axis, std::vector<Point>& queue, unsigned k) const {
    
        int next;
        int other;
        double currentDistance = (point - nodes[index].location).squaredNorm();
        if ( (axis && lessX(point,nodes[index].location)) || (!axis && lessY(point,nodes[index].location))) {
            next = nodes[index].left;
            other = nodes[index].right;
        } else {
            next = nodes[index].right;
            other = nodes[index].left;
        }
        if (next==-1) {
            queue.emplace_back(index,currentDistance);
            
        } else {
            searchNearest(next,point, !axis, queue, k);
            if (queue.size()<k) {
                queue.emplace_back(index,currentDistance);
            } else if (currentDistance < queue.back().distance) {
                queue.back().index = index;
                queue.back().distance = currentDistance;
            }
            std::sort(queue.begin(),queue.end());
        }
        if (other != -1) {
            double distanceToPlane = axis ? (point.getX() - nodes[index].location.getX())*(point.getX() - nodes[index].location.getX()) :
                (point.getY() - nodes[index].location.getY())*(point.getY() - nodes[index].location.getY());
            if (queue.size()<k || distanceToPlane < queue.back().distance) {
                std::vector<Point> otherQueue;
                searchNearest(other,point,!axis, otherQueue, k);
                for (unsigned i=0;i<otherQueue.size();i++) {
                    queue.push_back(otherQueue[i]);
                }
                std::sort(queue.begin(),queue.end());
                if (queue.size()>k) {
                    queue.resize(k);
                }
                
            }
        }
    }

    int searchNearest(int index, const utils::Vector2d& point, 
         double& distance, bool axis) const {
        int next;
        int other;
        double currentDistance = (point - nodes[index].location).squaredNorm();
        if ( (axis && lessX(point,nodes[index].location)) || (!axis && lessY(point,nodes[index].location))) {
            next = nodes[index].left;
            other = nodes[index].right;
        } else {
            next = nodes[index].right;
            other = nodes[index].left;
        }
        if (next==-1) {
            distance = currentDistance;
            return index;
        } 
        int best = searchNearest(next,point,distance,!axis);
        if (currentDistance < distance ) {
            distance = currentDistance;
            best = index;
        }
        if (other != -1) {
            double distanceToPlane = axis ? (point.getX() - nodes[index].location.getX())*(point.getX() - nodes[index].location.getX()) :
                (point.getY() - nodes[index].location.getY())*(point.getY() - nodes[index].location.getY());
            if (distanceToPlane < distance) {
                double otherDistance=0;
                int otherBest = searchNearest(other,point,otherDistance,!axis);
                if (otherDistance < distance) {
                    distance = otherDistance;
                    best = otherBest;
                }
            }
        }
        return best;
    }

    std::vector<Node> nodes;

};
}