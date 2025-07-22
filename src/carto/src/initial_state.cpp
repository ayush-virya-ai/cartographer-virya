#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"

#include <memory>
#include <string>

// It subscribes to /initialpose and uses the received pose to call the
// /start_trajectory service in Cartographer. It also finishes a previous
// trajectory upon startup.
class StartTrajectoryFromInitialPose : public rclcpp::Node {
public:
    StartTrajectoryFromInitialPose() : Node("start_trajectory_from_initial_pose") {
        RCLCPP_INFO(this->get_logger(), "Node starting up...");

        // Define configuration paths
        config_directory_ = "/home/ayush/Documents/ros_ws/src/transformations/config/";
        config_basename_ = "localisation.lua";

        // Create a subscriber to the /initialpose topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&StartTrajectoryFromInitialPose::initial_pose_callback, this, std::placeholders::_1));

        // Create clients for the Cartographer services
        finish_trajectory_client_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("/finish_trajectory");
        start_trajectory_client_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("/start_trajectory");

        // Finish the previous trajectory (e.g., trajectory_id = 1)
        finish_previous_trajectory(2);

        RCLCPP_INFO(this->get_logger(), "Node started. Waiting for an initial pose on /initialpose...");
    }

private:
    // Callback function for the /initialpose subscriber
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received initial pose. Calling /start_trajectory service...");

        // Wait for the /start_trajectory service to be available
        if (!start_trajectory_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "/start_trajectory service not available. Aborting.");
            return;
        }

        // Create and populate the request
        auto request = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
        request->configuration_directory = config_directory_;
        request->configuration_basename = config_basename_;
        request->use_initial_pose = true;
        request->initial_pose = msg->pose.pose;
        request->relative_to_trajectory_id = 0;

        // Asynchronously call the service
        auto future_response = start_trajectory_client_->async_send_request(request);

        // NOTE: In a real application, you would handle the future response,
        // for example, by spinning until the future is complete or using a callback.
        // For simplicity here, we just send the request.
        RCLCPP_INFO(this->get_logger(), "Request sent to /start_trajectory. Shutting down this node.");

        // Shutdown the node after sending the request as its job is done.
        rclcpp::shutdown();
    }

    // Method to call the /finish_trajectory service
    void finish_previous_trajectory(int trajectory_id) {
        // Wait for the service to become available
        while (!finish_trajectory_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "/finish_trajectory service not available, waiting...");
        }

        // Create and send the request
        auto request = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
        request->trajectory_id = trajectory_id;

        auto future = finish_trajectory_client_->async_send_request(request);
        
        // Wait for the service call to complete
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Successfully finished trajectory %d.", trajectory_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call /finish_trajectory service.");
        }
    }

    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_client_;
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_client_;
    std::string config_directory_;
    std::string config_basename_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StartTrajectoryFromInitialPose>();
    rclcpp::spin(node);
    // rclcpp::shutdown() is called within the node after its task is complete.
    return 0;
}
