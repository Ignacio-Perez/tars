#pragma once

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

class Tf2
{
public:
    ~Tf2() {}
    Tf2(const Tf2& other) = delete;
    void operator=(const Tf2& other) = delete;

    static Tf2& getInstance() {
        static Tf2 instance;
        return instance;
    }

    #define TF2 Tf2::getInstance()

    void init(rclcpp::Node::SharedPtr node);

    bool transform(
        const nav_msgs::msg::Odometry& msg,
        utils::Vector2d& pos,
        utils::Angle& yaw,
        const std::string& frame) const;

private:
    Tf2() = default;

    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
};

void Tf2::init(rclcpp::Node::SharedPtr node)
{
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
}

bool Tf2::transform(
    const nav_msgs::msg::Odometry& msg,
    utils::Vector2d& pos,
    utils::Angle& angle,
    const std::string& frame) const
{
    geometry_msgs::msg::PoseStamped poseIn;
    poseIn.header = msg.header;
    poseIn.pose = msg.pose.pose;

    geometry_msgs::msg::PoseStamped poseOut;

    try {
        poseOut = buffer->transform(
            poseIn,
            frame,
            tf2::durationFromSec(0.2));  // timeout
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("Tf2"), "%s", ex.what());
        return false;
    }

    pos.set(poseOut.pose.position.x, poseOut.pose.position.y);

    tf2::Quaternion q(
        poseOut.pose.orientation.x,
        poseOut.pose.orientation.y,
        poseOut.pose.orientation.z,
        poseOut.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    angle.setRadian(yaw);
    return true;
}