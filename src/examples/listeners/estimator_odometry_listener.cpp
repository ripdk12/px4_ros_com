
 #include <rclcpp/rclcpp.hpp>
 #include <px4_msgs/msg/estimator_odometry.hpp>

class EkfOdometryListener : public rclcpp::Node
{
public:
	explicit EkfOdometryListener() : Node("ekf_odometry_listener") {
		subscription_ = this->create_subscription<px4_msgs::msg::EstimatorOdometry>(
			"/fmu/estimator_odometry/out", 10,
			[this](const px4_msgs::msg::EstimatorOdometry::UniquePtr msg) {
			std::cout << "\n\n\n\n\n";
			std::cout << "ESTIMATED VISUAL ODOMETRY DATA"      << std::endl;
			std::cout << "============================="       << std::endl;
			std::cout << "ts: "         << msg->timestamp      << std::endl;
            std::cout << "x: "          << msg->x              << std::endl;
			std::cout << "y: "          << msg->y              << std::endl;
			std::cout << "z: "          << msg->z              << std::endl;
			std::cout << "q[x] "        << msg->q[0]           << std::endl;
			std::cout << "q[y]: "       << msg->q[1]           << std::endl;
			std::cout << "q[z]: "       << msg->q[2]           << std::endl;
			std::cout << "q[w]: "       << msg->q[3]           << std::endl;
			std::cout << "vx: "         << msg->vx             << std::endl;
			std::cout << "vy: "         << msg->vy             << std::endl;
			std::cout << "vz: "         << msg->vz             << std::endl;
			std::cout << "rollspeed: "  << msg->rollspeed      << std::endl;
			std::cout << "pitchspeed: " << msg->pitchspeed     << std::endl;
			std::cout << "yawspeed: "   << msg->yawspeed       << std::endl;
            std::cout << "pose frame: " << msg->local_frame    << std::endl;
            std::cout << "vel frame: "  << msg->velocity_frame << std::endl;
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::EstimatorOdometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting estimator visual odometry listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EkfOdometryListener>());

	rclcpp::shutdown();
	return 0;
}
