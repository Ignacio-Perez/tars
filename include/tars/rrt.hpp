#pragma once

#include <list>
#include <cmath>
#include "kdtree.hpp"
#include "navigation_map.hpp"


class RRT
{
public:
    RRT() : edgeLength(0.2), rrtStar(true) {}

    void init(const utils::Vector2d& root, double edgeLength, bool rrtStar=true);
    void iterate();
    
    void clear() {
        tree.clear();
        parents.clear();
    }

    bool getPath(const utils::Vector2d& dst, std::list<utils::Vector2d>& path) const;

private:
    void iterateRRT();
    void iterateRRTStar();

    double edgeLength;
    bool rrtStar;
    utils::KdTree tree;
    std::vector<unsigned> parents;
    std::vector<double> cost;
    

};

inline
bool RRT::getPath(const utils::Vector2d& dst, std::list<utils::Vector2d>& path) const {
    if (tree.size()==0) {
        return false;
    }
    unsigned index = tree.getNearest(dst);
    if ( (dst - tree[index]).norm() > edgeLength) {
        return false;
    }
    path.clear();
    while (index != 0) {
        path.push_front(tree[index]);
        index = parents[index]; 
    }
    return true;
}


inline
void RRT::init(const utils::Vector2d& root, double edgeLength, bool rrtStar) {
    tree.clear();
    parents.clear();
    cost.clear();
    cost.push_back(0);
    parents.push_back(0);
    tree.add(root);
    RRT::edgeLength = edgeLength;
    RRT::rrtStar = rrtStar;
}

inline
void RRT::iterate() {
    if (rrtStar) {
        iterateRRTStar();
    } else {
        iterateRRT();
    }
}


inline
void RRT::iterateRRT() {
    utils::Vector2d xrand;
    if (!NAV_MAP.sampleFree(xrand)) {
        return;
    }
    unsigned index = tree.getNearest(xrand);
    const utils::Vector2d& xnearest = tree[index];
    utils::Vector2d xnew = xnearest + edgeLength * (xrand-xnearest).normalized();
    if (NAV_MAP.isFree(xnearest,xnew)) {
        tree.add(xnew);
        parents.push_back(index);
    }

}

inline
void RRT::iterateRRTStar() {
    utils::Vector2d xrand;
    if (!NAV_MAP.sampleFree(xrand)) {
        return;
    }
    unsigned xnearestIndex = tree.getNearest(xrand);
    const utils::Vector2d& xnearest = tree[xnearestIndex];
    utils::Vector2d xnew = xnearest + edgeLength * (xrand-xnearest).normalized();
    if (NAV_MAP.isFree(xnearest,xnew)) {
        unsigned k = (unsigned)std::round(5.436563657 * std::log(tree.size()));
        std::vector<unsigned> indexes;
        tree.getKNearests(xnew,indexes,k);
        tree.add(xnew);
        unsigned xminIndex = xnearestIndex;
        double cmin = cost[xnearestIndex] + edgeLength;
        for (unsigned i=0;i<indexes.size();i++) {
            const utils::Vector2d& xnear = tree[indexes[i]];
            if (NAV_MAP.isFree(xnear,xnew) && cost[indexes[i]] + (xnear - xnew).norm() < cmin) {
                xminIndex = indexes[i];
                cmin = cost[indexes[i]] + (xnear - xnew).norm();
            }
        }
        parents.push_back(xminIndex);
        cost.push_back(cmin);
        for (unsigned i=0;i<indexes.size();i++) {
            const utils::Vector2d& xnear = tree[indexes[i]];
            if (NAV_MAP.isFree(xnew,xnear) && cmin + (xnew - xnear).norm() < cost[indexes[i]]) {
                parents[indexes[i]] = parents.size()-1;
            }
        }
    }
}
