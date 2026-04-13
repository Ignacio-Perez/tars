#include <list>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tars/msg/agents_msg.hpp"
#include "tars/srv/robot_goal_srv.hpp"
#include "tars/vector2d.hpp"
#include "tars/angle.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using Trigger = std_srvs::srv::Trigger;

class TarsSFMControl : public rclcpp::Node 
{
public:
	TarsSFMControl();
	void init();

private:
	void trackingCallback(const tars::msg::AgentsMsg& agents);
	void goalReceivedCallback(const geometry_msgs::msg::PoseStamped& msg);
	void control(); 
	void dwa();
	void publishGoals();
	void reset(const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response);

	std::string robotID; // Robot ID
	double robotRadius; // Robot radius
	double robotMaxVel; // Robot max velocity (m/s)
	bool trackingReceived = false; // True if at least one tracking message has been received
	
	utils::Vector2d robotPosition; // Robot position ("map" frame)
	utils::Angle robotYaw; // Robot yaw angle
	utils::Vector2d robotVelocity; // Robot velocity
	utils::Vector2d robotGlobalForce; // Robot global force (SFM)

	std::list<utils::Vector2d> goals; // goals FIFO list
	utils::Vector2d currentGoal; // current goal
	
	
	double K1;
	double K2;
	double dt;

	// Agents tracking subscription
  rclcpp::Subscription<tars::msg::AgentsMsg>::SharedPtr trackingSub;

  // Goal subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub;

  // cmd_vel publisher
 	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;

 	// Goals visualization publisher
 	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goalsListPub;

  // callback timer to control loop
	rclcpp::TimerBase::SharedPtr timer;
  
  // Goal client
	rclcpp::Client<tars::srv::RobotGoalSrv>::SharedPtr goalSrv;

	rclcpp::Service<Trigger>::SharedPtr resetSrv;

};

TarsSFMControl::TarsSFMControl() 
: Node("tars_sfm_control") {}

void TarsSFMControl::init() {

	trackingReceived = false;

	declare_parameter<std::string>("robot_id","r09");
	robotID = get_parameter("robot_id").as_string();

	declare_parameter<double>("robot_radius",0.2);
 	robotRadius = get_parameter("robot_radius").as_double();

 	declare_parameter<double>("robot_max_vel",0.6);
 	robotMaxVel = get_parameter("robot_max_vel").as_double();

 	declare_parameter<double>("K1",1.0);
 	K1 = get_parameter("K1").as_double();

 	declare_parameter<double>("K2",0);
 	K2 = get_parameter("K2").as_double();

	declare_parameter<std::string>("agents_tracking_topic","/tars/r09/agents_tracking");
	std::string trackingTopic = get_parameter("agents_tracking_topic").as_string();

	declare_parameter<std::string>("cmd_vel_topic","/tars/r09/cmd_vel");
	std::string cmdVelTopic = get_parameter("cmd_vel_topic").as_string();

	declare_parameter<std::string>("reset_service","/tars_sfm_control/r09/reset");
	std::string resetSrvName = get_parameter("reset_service").as_string();

	declare_parameter<std::string>("goal_topic","/goal_pose");
	std::string goalTopic = get_parameter("goal_topic").as_string();

	declare_parameter<std::string>("goals_list_topic","/tars_sfm_control/visualization/r09/goals");
	std::string goalsListTopic = get_parameter("goals_list_topic").as_string();

	declare_parameter<std::string>("goal_service","/tars/robot_goal");
	std::string goalService = get_parameter("goal_service").as_string();

	declare_parameter<double>("node_freq",100.0);
	double nodeFreq = get_parameter("node_freq").as_double();
	dt = 1.0 / nodeFreq;

	

	trackingSub = create_subscription<tars::msg::AgentsMsg>(trackingTopic, 1, std::bind(&TarsSFMControl::trackingCallback, this, _1));

	goalSub = create_subscription<geometry_msgs::msg::PoseStamped>(goalTopic,1, std::bind(&TarsSFMControl::goalReceivedCallback, this, _1));

	cmdVelPub = create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic,1);

	goalsListPub = create_publisher<visualization_msgs::msg::MarkerArray>(goalsListTopic,1);

	int period = std::round(1000.0 / nodeFreq);

  timer = create_wall_timer(std::chrono::milliseconds(period), std::bind(&TarsSFMControl::control,this));  

  goalSrv = create_client<tars::srv::RobotGoalSrv>(goalService);

  resetSrv = create_service<Trigger>(
            resetSrvName,
            std::bind(&TarsSFMControl::reset, this,
                      std::placeholders::_1,
                      std::placeholders::_2));
}

void TarsSFMControl::trackingCallback(const tars::msg::AgentsMsg& agents) {
	for (unsigned i=0;i<agents.size;i++) {
		if (agents.agents[i].id == robotID) {
			robotPosition.set(agents.agents[i].position.x, agents.agents[i].position.y);
			robotYaw.setRadian(agents.agents[i].yaw);
			robotVelocity.set(agents.agents[i].velocity.x, agents.agents[i].velocity.y);
			robotGlobalForce.set(agents.agents[i].forces.global_force.x,agents.agents[i].forces.global_force.y);
			break;
		}
	}
	trackingReceived = true;
}

void TarsSFMControl::goalReceivedCallback(const geometry_msgs::msg::PoseStamped& msg) {
	if (msg.header.frame_id != "map") {
		RCLCPP_WARN(get_logger(),"Invalid Goal frame_id '%s'",msg.header.frame_id.c_str());
		return;
	}
	utils::Vector2d goal(msg.pose.position.x, msg.pose.position.y);
	goals.push_back(goal);
	RCLCPP_INFO(get_logger(),"Goal received (%f, %f)",goal.getX(),goal.getY());
}

void TarsSFMControl::reset(const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response) {

	RCLCPP_INFO(get_logger(), "RESET");
	geometry_msgs::msg::Twist cmdVel;
	cmdVel.linear.x = 0;
  cmdVel.angular.z = 0;
  cmdVelPub->publish(cmdVel);

  auto r = std::make_shared<tars::srv::RobotGoalSrv::Request>();
	r->id = robotID;
	r->enable_forces = false;
	goalSrv->async_send_request(r); 

  trackingReceived = false;
  goals.clear();
  (void)request;
  response->success = true;
  response->message = "SUCCESS";

}

void TarsSFMControl::control() {

	publishGoals();

	// Remove reached goals
	while (trackingReceived &&
		   !goals.empty() && 
		    (robotPosition - goals.front()).norm() < 2.0 * robotRadius) {
		RCLCPP_INFO(get_logger(),"Goal reached (%f, %f)",goals.front().getX(),goals.front().getY());
		goals.pop_front();
	}

	// If no tracking received, 
	// or the goal service is not ready, 
	// or goals are empty:
	// stop the robot and return
	if (!trackingReceived || 
		!goalSrv->service_is_ready() ||
		goals.empty()) {
		geometry_msgs::msg::Twist cmdVel;
		cmdVel.linear.x = 0;
        cmdVel.angular.z = 0;
        cmdVelPub->publish(cmdVel);
		return;
	}

	// Here, we have tracking received, available goal service, and at least one unreached goal 

	// If the current goal has changed, call the goal service
	if (currentGoal != goals.front()) {
		auto request = std::make_shared<tars::srv::RobotGoalSrv::Request>();
		request->id = robotID;
		request->enable_forces = true;
		request->gx = goals.front().getX();
		request->gy = goals.front().getY();
		auto future = goalSrv->async_send_request(request); 
		currentGoal = goals.front();
	}

	// call Dynamic Window Approach algorithm
	dwa();

}

// Algoritmo Dynamic Window Approach que valora una serie de trayectorias circulares y ejecuta la mas apropiada
// de acuerdo a unos criterios. En esta implementacion, el criterio es sencillo, elige la trayectoria que mejor
// aproxime la velocidad instantanea deseada de acuerdo al vector de fuerza global
void TarsSFMControl::dwa() {
	
	// Vector de posibles trayectorias circulares
	static std::vector<geometry_msgs::msg::Twist> commands;
	if (commands.empty()) {
		for (double lin = 0; lin<= 0.6; lin+=0.01) {
			for (double ang = -0.8; ang<=0.8; ang+=0.01) {
				geometry_msgs::msg::Twist cmdVel;
				cmdVel.linear.x = lin;
				cmdVel.angular.z = ang;
				commands.push_back(cmdVel);
			}
		}
	}


	// Calculamos el vector de referencia de velocidad instantanea 
	utils::Vector2d velocityRef = robotVelocity + robotGlobalForce * dt;

	// Si el vector de referencia excede la velocidad maxima, ajustamos
	if (velocityRef.norm() > robotMaxVel) {
		velocityRef.normalize();
		velocityRef *= robotMaxVel;
	}
	
	// Calculamos la pose que tendria el robot si pudiera seguir el vector de referencia por un tiempo dt
	utils::Vector2d posRef = robotPosition + velocityRef * dt;
	utils::Angle yawRef = velocityRef.angle();

	
	// Para cada trayectoria circular, calculamos la pose resultante tras un tiempo dt 
	// y comparamos con la pose de referencia
	double minDist = 999999999;
	geometry_msgs::msg::Twist cmdVel;
	for (unsigned i=0;i<commands.size();i++) {
		double imd = commands[i].linear.x * dt;
		utils::Vector2d inc(imd * std::cos(robotYaw.toRadian() + commands[i].angular.z*dt*0.5), 
			                imd * std::sin(robotYaw.toRadian() + commands[i].angular.z*dt*0.5));
		utils::Vector2d pos = robotPosition + inc;
		utils::Angle yaw = robotYaw + utils::Angle::fromRadian(commands[i].angular.z * dt);	

		double dist = K1*(posRef - pos).norm() + K2 * fabs((yaw - yawRef).toRadian()); 
		if (dist < minDist) {
			minDist = dist;
			cmdVel = commands[i];
		}
	}
	cmdVelPub->publish(cmdVel);

}

void TarsSFMControl::publishGoals()
{
    visualization_msgs::msg::MarkerArray markers;
    unsigned counter = 0;
    int goalId = 1;

    for (auto it = goals.begin(); it != goals.end(); ++it) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = get_clock()->now();

        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = counter++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        marker.pose.position.x = it->getX();
        marker.pose.position.y = it->getY();
        marker.pose.position.z = 0.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        markers.markers.push_back(marker);

        marker.id = counter++;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.pose.position.z = 0.2;
        marker.text = std::to_string(goalId++);

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        markers.markers.push_back(marker);
    }

    goalsListPub->publish(markers);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TarsSFMControl>();
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
