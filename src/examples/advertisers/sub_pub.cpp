#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdometryListener : public rclcpp::Node
{
public:
	OdometryListener() : Node("Odometry_listener")
	{
		publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in", 10);
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"visual_slam/tracking/odometry", 10,
			[this](const nav_msgs::msg::Odometry::UniquePtr msg)
			{
				auto odometry_msg = px4_msgs::msg::VehicleVisualOdometry();

				odometry_msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

				odometry_msg.x = msg->pose.pose.position.x;
				odometry_msg.y = msg->pose.pose.position.y;
				odometry_msg.z = msg->pose.pose.position.z;

				odometry_msg.q[0] = msg->pose.pose.orientation.x;
				odometry_msg.q[1] = msg->pose.pose.orientation.y;
				odometry_msg.q[2] = msg->pose.pose.orientation.z;
				odometry_msg.q[3] = msg->pose.pose.orientation.w;

				double poseMatrix[6][6] = {};
				double velMatrix[6][6] = {};
				int x = 0;

				for (int r = 0; r < 6; ++r){
					for (int c = 0; c < 6; ++c){
						poseMatrix[r][c] = msg->pose.covariance[x];
						velMatrix[r][c] = msg->twist.covariance[x];
						x++;
					}
				}
				//int i, j;
				x = 0;
				int row = 6, col = 6;

				for (int i = 0; i < row; i++){
					for (int j = 0; j < col; j++){
						if (i <= j){
							odometry_msg.pose_covariance[x] = poseMatrix[i][j];
							odometry_msg.velocity_covariance[x] = velMatrix[i][j];
							x++;
						}
					}
				}

				odometry_msg.vx = msg->twist.twist.linear.x;
				odometry_msg.vy = msg->twist.twist.linear.y;
				odometry_msg.vz = msg->twist.twist.linear.z;

				odometry_msg.rollspeed = msg->twist.twist.angular.x;
				odometry_msg.pitchspeed = msg->twist.twist.angular.y;
				odometry_msg.yawspeed = msg->twist.twist.angular.z;

				odometry_msg.local_frame = 1;
				odometry_msg.velocity_frame = 1;

				this->publisher_->publish(odometry_msg);
			});
	}

private:
	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting odometry message translation node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdometryListener>());

	rclcpp::shutdown();
	return 0;
}

