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

#ifndef _KDTREE_HPP_
#define _KDTREE_HPP_

#include <algorithm>
#include <vector>
#include "vector2d.hpp"

namespace utils
{

class KdTree
{
public:
	KdTree(std::vector<utils::Vector2d>& points) 
	: root(createKdTree(points,0,points.size(),true)) {}
	~KdTree() {
		destroyKdTree(root);
	}

	double getDistanceToNearestPoint(const utils::Vector2d& point) const {
		if (root==NULL) {
			return std::numeric_limits<double>::infinity();
		}
		double distance = 0;
		searchNearest(root,point,distance,true);
		return distance;
	}

	const utils::Vector2d& getNearestPoint(const utils::Vector2d& point) const {
		double distance=0;
		return searchNearest(root,point,distance,true)->location;
	}

private:
	struct Node {
		Node(const utils::Vector2d& location) : location(location), left(NULL), right(NULL) {}
		utils::Vector2d location;
		Node *left;
		Node *right;
	};

	static bool lessX(const utils::Vector2d& a, const utils::Vector2d& b) {
		return a.getX() < b.getX();
	}
	static bool lessY(const utils::Vector2d& a, const utils::Vector2d& b) {
		return a.getY() < b.getY();
	}

	static Node* createKdTree(std::vector<utils::Vector2d>& points, int begin, int end, bool axis) {
		if (end <= begin) {
			return NULL;
		}
		if (axis) {
			std::sort(points.begin()+begin, points.begin()+end, lessX);
		} else {
			std::sort(points.begin()+begin, points.begin()+end, lessY);
		}
		int median = begin + (end-begin)/2;
		Node* tree = new Node(points[median]);
		tree->left = createKdTree(points,begin,median,!axis);
		tree->right = createKdTree(points,median+1,end,!axis);
		return tree;
	}

	static void destroyKdTree(Node* node) {
		if (node != NULL) {
			destroyKdTree(node->left);
			destroyKdTree(node->right);
		}
		delete node;
	}

	Node* searchNearest(Node* tree, const utils::Vector2d& point, 
		 double& distance, bool axis) const {
		Node* next;
		Node* other;
		double currentDistance = (point - tree->location).squaredNorm();
		if ( (axis && lessX(point,tree->location)) || (!axis && lessY(point,tree->location))) {
			next = tree->left;
			other = tree->right;
		} else {
			next = tree->right;
			other = tree->left;
		}
		if (next==NULL) {
			distance = currentDistance;
			return tree;
		} 
		Node* best = searchNearest(next,point,distance,!axis);
		if (currentDistance < distance ) {
			distance = currentDistance;
			best = tree;
		}
		if (other != NULL) {
			double distanceToPlane = axis ? (point.getX() - tree->location.getX())*(point.getX() - tree->location.getX()) :
		  		(point.getY() - tree->location.getY())*(point.getY() - tree->location.getY());
			if (distanceToPlane < distance) {
				double otherDistance=0;
				Node* otherBest = searchNearest(other,point,otherDistance,!axis);
				if (otherDistance < distance) {
					distance = otherDistance;
					best = otherBest;
				}
			}
		}
		return best;
	}
	Node* root;
};
}
#endif