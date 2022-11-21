
 #include <rclcpp/rclcpp.hpp>
 #include <px4_msgs/msg/estimator_visual_odometry_aligned.hpp>

class EkfOdometryAlignedListener : public rclcpp::Node
{
public:
	explicit EkfOdometryAlignedListener() : Node("ekf_odometry_listener") {
		subscription_ = this->create_subscription<px4_msgs::msg::EstimatorVisualOdometryAligned>(
			"/fmu/estimator_visual_odometry_aligned/out", 10,
			[this](const px4_msgs::msg::EstimatorVisualOdometryAligned::UniquePtr msg) {
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
	rclcpp::Subscription<px4_msgs::msg::EstimatorVisualOdometryAligned>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting visual odometry listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EkfOdometryAlignedListener>());

	rclcpp::shutdown();
	return 0;
}
