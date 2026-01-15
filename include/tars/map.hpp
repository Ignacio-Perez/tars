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

#ifndef _MAP_HPP_
#define _MAP_HPP_

#include <fstream>
#include <string>
#include "vector2d.hpp"
#include "kdtree.hpp"

class Map
{
public:
	Map() : resolution(0.05), width(0), height(0), kdTree(NULL) {}
	~Map() {if (kdTree!=NULL) delete kdTree;}
	void load(const std::string& workingDirectory, const std::string& yaml);	
	double getDistanceToNearestObstacle(const utils::Vector2d& point) const;
	utils::Vector2d getNearestObstacle(const utils::Vector2d& point) const;
	bool isObstacle(const utils::Vector2d& point) const;
	bool isReachableInStraightLine(const utils::Vector2d& a, const utils::Vector2d& b) const;
	double getResolution() const {return resolution;}
	double getWidth() const {return width;}
	double getHeight() const {return height;}
	bool isLoaded() const {return kdTree!=NULL;}
private:
	static std::fstream getFile(const std::string& path, const std::string& yaml, std::string& fileName);
	static double stod(const std::string& yaml);
	static int stoi(const std::string& yaml);
	static char* readPGM(const std::string& path, const std::string& line, int& width, int& height, int& maxval);
	static unsigned readPGMLine(std::fstream& file, std::string& buffer);
	double resolution;
	double width;
	double height;
	utils::KdTree* kdTree;
	std::vector<std::vector<bool>> occupancy;
};

inline	
bool Map::isReachableInStraightLine(const utils::Vector2d& a, const utils::Vector2d& b) const {
	utils::Vector2d u = (b - a).normalized();
	u *= resolution/2;
	utils::Vector2d x = a;
	while ((b-x).norm()>resolution) {
		if (getDistanceToNearestObstacle(x)<0.2 ) {
			return false;
		}
		x+=u;
	}
	return true;
}

inline
double Map::getDistanceToNearestObstacle(const utils::Vector2d& point) const {
	if (kdTree==NULL) {
		return std::numeric_limits<double>::infinity();
	}
	double distance = kdTree->getDistanceToNearestPoint(point);
	double d = std::sqrt(distance) - resolution/2;
	if (d < 0) {
		d = 0;
	}
	return d;
}

inline	
utils::Vector2d Map::getNearestObstacle(const utils::Vector2d& point) const {
	utils::Vector2d best;
	if (kdTree!=NULL) {
		best = kdTree->getNearestPoint(point);
	}
	return best;
}

inline
bool Map::isObstacle(const utils::Vector2d& point) const {
	int x = std::floor(point.getX()/resolution);
	int y = std::floor(point.getY()/resolution);
	if (y<0 || (unsigned)y >= occupancy.size() || x<0 || (unsigned)x >= occupancy[0].size() ) {
		return true;
	}
	return occupancy[y][x];
}

inline
double Map::stod(const std::string& yaml) {
	return std::stod(yaml.substr(yaml.find_first_of(':')+1));
}

inline	
int Map::stoi(const std::string& yaml) {
	return std::stoi(yaml.substr(yaml.find_first_of(':')+1));
}

inline
char* Map::readPGM(const std::string& path, const std::string& line, int& width, int& height, int& maxval) {
	std::string fileName;
	std::fstream file = getFile(path,line,fileName);
	std::string buffer;
	unsigned counter = 0;
	counter += readPGMLine(file,buffer);
	if (buffer!="P5") {
		throw std::runtime_error("Invalid PGM file");	
	}
	counter += readPGMLine(file,buffer);
	width = std::stoi(buffer);
	counter += readPGMLine(file,buffer);
	height = std::stoi(buffer);
	counter += readPGMLine(file,buffer);
	maxval = std::stoi(buffer);
	if (width<=0 || height<=0 || maxval <=0) {
		throw std::runtime_error("Invalid PGM file");
	}
	if (maxval > 255) {
		throw std::runtime_error("Unsupported maxval in PGM file");
	}
	file.close();
	std::ifstream is;
	is.open(fileName, std::ios::binary | std::ios::in);
	is.seekg(counter);
	char* raster = new char[width*height];
	is.read(raster,width*height);
	is.close();
	return raster;
}

inline
unsigned Map::readPGMLine(std::fstream& file, std::string& buffer) {
	unsigned counter = 1;
	int ch = file.get();
	while (ch == '#') {
		while (!file.eof() && file.get()!='\n') {
			++counter;
		}
		ch = file.get();
		counter+=2;
	}
	if (file.eof()) {
		throw std::runtime_error("Invalid PGM file");	
	}
	buffer.clear();
	buffer.push_back(ch);
	while (!file.eof()) {
		ch = file.get();
		++counter;
		if (std::isspace(ch)) {
			break;
		}
		buffer.push_back(ch);
	}
	if (file.eof()) {
		throw std::runtime_error("Invalid PGM file");	
	}
	return counter;
}

inline
std::fstream Map::getFile(const std::string& path, const std::string& yaml, std::string& fileName) {
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
	fileName = path + "/" + yaml.substr(pos);
	unsigned length = fileName.size();
	while (length>0 && std::isspace(fileName[length-1])) {
		--length;
	}
	fileName = fileName.substr(0,length);
	std::fstream file(fileName,std::ios::in);
	if (!file.is_open()) {
		throw std::runtime_error("Cannot open file '" + fileName+"'");
	}
	return file;
}

inline
void Map::load(const std::string& workingDirectory, const std::string& yaml) {
	if (kdTree != NULL) {
		throw std::runtime_error("Map is already defined");
	}
	std::string fileName;
	std::fstream file = getFile(workingDirectory,yaml,fileName);
	std::string line;
	double freeThresh = 0.196;
	resolution = 0.05;
	int negate = 0;
	int width = 0;
	int height = 0;
	int maxval = 0;
	char* raster = NULL;
	while (std::getline(file,line)) {
		if (line.size()==0) {
			continue;
		}
		if (line.find("image:")==0) {
			raster = readPGM(workingDirectory,line,width,height,maxval);
		} else if (line.find("resolution:")==0) {
			resolution = stod(line);
		} else if (line.find("negate:")==0) {
			negate = stoi(line);
		} else if (line.find("free_thresh:")==0) {
			freeThresh = stod(line);
		} else if (line.find("origin:")==0) {
			if (line != "origin: [0.000000, 0.000000, 0.000000]") {
				throw std::runtime_error("Unsupported origin in map file: "+line);
			}
		}
	}
	file.close();
	if (raster == NULL) {
		throw std::runtime_error("Invalid PGM file");
	}
	std::vector<utils::Vector2d> obstacles;
	occupancy.resize(height);
	double x,y = 0;
	for (int j=height-1;j>=0;j--) {
		occupancy[height-1-j].resize(width);
		x = 0;
		for (int i=0;i<width;i++) {
			int ch = (unsigned char)raster[j*width + i];
			double p = negate ? (double)ch / (double)maxval : (double)(maxval - ch) / (double)maxval;
			if (p >= freeThresh) {
				obstacles.emplace_back(x,y);
				occupancy[height-1-j][i] = true;
			} 
			x += resolution;
		}
		y += resolution;
	}
	delete raster;
	kdTree = new utils::KdTree(obstacles);
	Map::width = (double)width * resolution;
	Map::height = (double)height * resolution;
}
#endif