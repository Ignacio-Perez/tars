#pragma once

#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "random.hpp"
#include "vector2d.hpp"

class NavigationMap
{
public:
    ~NavigationMap() {}
    NavigationMap(const NavigationMap& other) = delete;
    void operator=(const NavigationMap& other) = delete;
    static NavigationMap& getInstance() {static NavigationMap instance; return instance;}
    #define NAV_MAP NavigationMap::getInstance()

    void setData(const nav_msgs::msg::OccupancyGrid& msg, double inflateRadius);
    const nav_msgs::msg::OccupancyGrid& getMap() const {return map;}

    bool sampleFree(utils::Vector2d& pos) const;
    bool isFree(const utils::Vector2d& pos) const;
    bool isFree(const utils::Vector2d& src, const utils::Vector2d& dst) const;
    bool isInitiated() const {return !freeCells.empty();}

private:
    NavigationMap();

    nav_msgs::msg::OccupancyGrid map;
    std::vector<unsigned> freeCells;
};

inline
NavigationMap::NavigationMap()  {}

inline 
bool NavigationMap::isFree(const utils::Vector2d& src, const utils::Vector2d& dst) const {
    if (src == dst) {
        return isFree(src);
    }
    if (!isFree(src) || !isFree(dst)) {
        return false;
    }
    utils::Vector2d u = dst;
    u -= src;
    double length = u.norm();
    u /= length;
    u *= map.info.resolution;

    utils::Vector2d x = src + u;
    double dist = map.info.resolution;

    while (dist < length) {
        if (!isFree(x)) {
            return false;
        }
        x += u;
        dist += map.info.resolution;
    }
    return true;
}

inline 
bool NavigationMap::sampleFree(utils::Vector2d& pos) const {
    if (freeCells.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("navigation_map"), "Cannot sample free space");
        return false;
    }

    unsigned cell = freeCells[RANDOM(freeCells.size())];
    unsigned i = cell % map.info.width;
    unsigned j = cell / map.info.width;

    double x = i * map.info.resolution + RANDOM() * map.info.resolution;
    double y = j * map.info.resolution + RANDOM() * map.info.resolution;

    pos.set(x, y);
    return true;
}

inline
bool NavigationMap::isFree(const utils::Vector2d& pos) const {
    int i = (int)std::floor(pos.getX() / map.info.resolution);
    int j = (int)std::floor(pos.getY() / map.info.resolution);

    if (i < 0 || i >= (int)map.info.width || j < 0 || j >= (int)map.info.height) {
        return false;
    }
    return map.data[j * map.info.width + i] == 0;
}

inline
void NavigationMap::setData(const nav_msgs::msg::OccupancyGrid& msg, double inflateRadius) {
    map.header = msg.header;
    map.info = msg.info;

    map.header.stamp = rclcpp::Clock().now();
    map.info.map_load_time = rclcpp::Clock().now();

    map.data.assign(map.info.width * map.info.height, 0);
    freeCells.clear();

    int offset = std::ceil(inflateRadius / map.info.resolution) + 1;

    double x0 = map.info.resolution / 2;
    double y0 = map.info.resolution / 2;

    for (int j = 0; j < (int)map.info.height; j++) {
        for (int i = 0; i < (int)map.info.width; i++) {

            signed char data = msg.data[j * map.info.width + i];
            if (data < 0 || data > 50) {
                map.data[j * map.info.width + i] = 100;

                for (int a = j - offset; a < j + offset; a++) {
                    if (a < 0 || a >= (int)map.info.height) continue;

                    double y1 = a * map.info.resolution + map.info.resolution / 2;

                    for (int b = i - offset; b < i + offset; b++) {
                        if (b < 0 || b >= (int)map.info.width) continue;

                        double x1 = b * map.info.resolution + map.info.resolution / 2;
                        double dist = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));

                        if (dist <= inflateRadius) {
                            map.data[a * map.info.width + b] = 100;
                        }
                    }
                }
            }
            x0 += map.info.resolution;
        }
        x0 = map.info.resolution / 2;
        y0 += map.info.resolution;
    }

    for (unsigned i = 0; i < map.data.size(); i++) {
        if (map.data[i] == 0) {
            freeCells.push_back(i);
        }
    }
}