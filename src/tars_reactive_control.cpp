#include <chrono>
#include <string>
#include "tars/random.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

// Robot state
enum State {INIT, MOVING_FORWARD, ROTATION_LEFT, ROTATION_RIGHT};

class TarsReactiveControl : public rclcpp::Node 
{
public:
  TarsReactiveControl();
  void init();

private:
  void control();

  void scanReceivedCallback(const sensor_msgs::msg::LaserScan& scan);

  double robotRadius;  // Robot radius (default 0.2m)
  double robotLinVel;  // Robot linear velocity (default 0.6 m/s)
  double robotAngVel;  // Robot angular velocity (default 0.5 rad/s)

  State state = State::INIT; // Robot state 
  bool scanReceived = false; // True if at least one scan message has been receive
  bool imminentCollision = false; // True if the robot is about to collide
 
  // callback timer to control loop
  rclcpp::TimerBase::SharedPtr timer;

  // Lidar scan subscription
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub;
  
  // cmd_vel publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;
};

TarsReactiveControl::TarsReactiveControl()
: Node("tars_reactive_control") {}

void TarsReactiveControl::init() {
 
  state = INIT;
  scanReceived = false;
  imminentCollision = false;

  declare_parameter<std::string>("scan_topic","/tars/r09/scan");
  std::string scanTopic = get_parameter("scan_topic").as_string();
  
  declare_parameter<std::string>("cmd_vel_topic","/tars/r09/cmd_vel");
  std::string cmdVelTopic = get_parameter("cmd_vel_topic").as_string();

  declare_parameter<double>("node_freq",30.0);
  double nodeFreq = get_parameter("node_freq").as_double();

  declare_parameter<double>("robot_radius",0.2);
  robotRadius = get_parameter("robot_radius").as_double();

  declare_parameter<double>("robot_linear_velocity",0.6);
  robotLinVel = get_parameter("robot_linear_velocity").as_double();

  declare_parameter<double>("robot_angular_velocity",0.5);
  robotAngVel = get_parameter("robot_angular_velocity").as_double();

  scanSub = create_subscription<sensor_msgs::msg::LaserScan>(scanTopic, 1, std::bind(&TarsReactiveControl::scanReceivedCallback, this, _1));

  cmdVelPub = create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic,1);

  int period = std::round(1000.0 / nodeFreq);

  timer = create_wall_timer(std::chrono::milliseconds(period), std::bind(&TarsReactiveControl::control,this));  
}

void TarsReactiveControl::scanReceivedCallback(const sensor_msgs::msg::LaserScan& scan) {
  imminentCollision = false;
  double angleIncrement = scan.angle_increment;
  double angle = scan.angle_min;
  for (unsigned i=0;i<scan.ranges.size();i++) {
    if (angle >= -M_PI/4 && angle <= M_PI/4 && scan.ranges[i]<2*robotRadius) {
      imminentCollision = true;
      break;
    }
    angle += angleIncrement;
  }
  scanReceived = true;  
}

/*****************/
/* ROBOT CONTROL */
/*****************/
void TarsReactiveControl::control() {
  switch (state) {
    case INIT:
      if (scanReceived) {
        state = MOVING_FORWARD;
        RCLCPP_INFO(get_logger(), "MOVING FORWARD");
      }
    break;

    case MOVING_FORWARD:
      if (imminentCollision) {
        if (RANDOM(2)==0) {
          state = ROTATION_LEFT;
          RCLCPP_INFO(get_logger(), "ROTATION LEFT");
        } else {
          state = ROTATION_RIGHT;
          RCLCPP_INFO(get_logger(), "ROTATION RIGHT");
        }
      } else {
        geometry_msgs::msg::Twist cmdVel;
        cmdVel.linear.x = robotLinVel;
        cmdVelPub->publish(cmdVel);
      }
    break;

    case ROTATION_LEFT:
    case ROTATION_RIGHT:
      if (!imminentCollision) {
        state = MOVING_FORWARD;
        RCLCPP_INFO(get_logger(), "MOVING FORWARD");
      } else {
        geometry_msgs::msg::Twist cmdVel;
        cmdVel.angular.z = state==ROTATION_LEFT ? robotAngVel : -robotAngVel;
        cmdVelPub->publish(cmdVel);
      }
    break;
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TarsReactiveControl>();
  try {
    node->init();
    rclcpp::spin(node);
  } catch(const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());    
    rclcpp::shutdown();
    return 1;
  } catch(...) {
    RCLCPP_FATAL(node->get_logger(), "unknown error, node aborted");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
