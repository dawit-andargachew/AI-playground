#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "six_ex4/srv/burrow_status.hpp"
#include <cmath>
#include <vector>

class TurtlebotNode : public rclcpp::Node
{
public:
    TurtlebotNode() : Node("turtlebot_node")
    {
        // Subscribe to LIDAR topic
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TurtlebotNode::lidar_callback, this, std::placeholders::_1));
        
        // Publisher for apple poses
        apple_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/apples", 10);
        
        // Create service server
        service_ = this->create_service<six_ex4::srv::BurrowStatus>(
            "check_burrow_status",
            std::bind(&TurtlebotNode::handle_service_request, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Turtlebot node initialized - Service server ready");
        RCLCPP_INFO(this->get_logger(), "Waiting for LIDAR data and service requests...");
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
        detect_apples();
    }
    
    void detect_apples()
    {
        if (!latest_scan_) {
            return;
        }
        
        detected_apples_.clear();
        
        // Parameters for apple detection
        const double MIN_DISTANCE = 0.1;  // Minimum valid distance (m)
        const double MAX_DISTANCE = 5.0;  // Maximum detection range (m)
        const double APPLE_CLUSTER_THRESHOLD = 0.15;  // Distance to group points as one apple
        const int MIN_POINTS_PER_APPLE = 3;  // Minimum consecutive points for an apple
        
        std::vector<std::pair<double, double>> current_cluster;
        
        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i)
        {
            double range = latest_scan_->ranges[i];
            
            // Skip invalid readings
            if (std::isnan(range) || std::isinf(range) || 
                range < MIN_DISTANCE || range > MAX_DISTANCE)
            {
                // End current cluster if we have one
                if (current_cluster.size() >= MIN_POINTS_PER_APPLE)
                {
                    add_apple_from_cluster(current_cluster);
                }
                current_cluster.clear();
                continue;
            }
            
            // Calculate angle for this reading
            double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
            
            // Convert polar to Cartesian (relative to base_link)
            double x = range * cos(angle);
            double y = range * sin(angle);
            
            // Check if this point belongs to current cluster
            if (!current_cluster.empty())
            {
                auto& last_point = current_cluster.back();
                double dist = sqrt(pow(x - last_point.first, 2) + pow(y - last_point.second, 2));
                
                if (dist < APPLE_CLUSTER_THRESHOLD)
                {
                    current_cluster.push_back({x, y});
                }
                else
                {
                    // Cluster ended, check if it's a valid apple
                    if (current_cluster.size() >= MIN_POINTS_PER_APPLE)
                    {
                        add_apple_from_cluster(current_cluster);
                    }
                    current_cluster.clear();
                    current_cluster.push_back({x, y});
                }
            }
            else
            {
                current_cluster.push_back({x, y});
            }
        }
        
        // Check last cluster
        if (current_cluster.size() >= MIN_POINTS_PER_APPLE)
        {
            add_apple_from_cluster(current_cluster);
        }
    }
    
    void add_apple_from_cluster(const std::vector<std::pair<double, double>>& cluster)
    {
        // Calculate centroid of cluster
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& point : cluster)
        {
            sum_x += point.first;
            sum_y += point.second;
        }
        
        double centroid_x = sum_x / cluster.size();
        double centroid_y = sum_y / cluster.size();
        
        // Create pose for this apple
        geometry_msgs::msg::Pose apple_pose;
        apple_pose.position.x = centroid_x;
        apple_pose.position.y = centroid_y;
        apple_pose.position.z = 0.0;
        apple_pose.orientation.w = 1.0;
        
        detected_apples_.push_back(apple_pose);
    }
    
    void handle_service_request(
        const std::shared_ptr<six_ex4::srv::BurrowStatus::Request> request,
        std::shared_ptr<six_ex4::srv::BurrowStatus::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "SERVICE REQUEST RECEIVED");
        RCLCPP_INFO(this->get_logger(), "Burrow status:");
        RCLCPP_INFO(this->get_logger(), "  - Current apples in burrow: %d", request->current_apples);
        RCLCPP_INFO(this->get_logger(), "  - Burrow capacity: %d", request->burrow_size);
        RCLCPP_INFO(this->get_logger(), "  - Apples needed to fill: %d", 
                   request->burrow_size - request->current_apples);
        
        // Detect apples from latest scan
        if (latest_scan_)
        {
            detect_apples();
        }
        
        int apples_found = detected_apples_.size();
        int apples_needed = request->burrow_size - request->current_apples;
        
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "LIDAR ANALYSIS:");
        RCLCPP_INFO(this->get_logger(), "  - Apples detected: %d", apples_found);
        
        // Publish detected apple poses
        if (!detected_apples_.empty())
        {
            geometry_msgs::msg::PoseArray pose_array;
            pose_array.header.stamp = this->now();
            pose_array.header.frame_id = "base_link";
            pose_array.poses = detected_apples_;
            apple_pub_->publish(pose_array);
            
            RCLCPP_INFO(this->get_logger(), "  - Published apple poses to /apples topic");
        }
        
        // Determine response
        bool enough = apples_found >= apples_needed;
        response->enough_apples = enough;
        response->apples_found = apples_found;
        
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "SERVICE RESPONSE:");
        if (enough)
        {
            RCLCPP_INFO(this->get_logger(), "  ✓ SUCCESS: Found enough apples to refill burrow!");
            RCLCPP_INFO(this->get_logger(), "  - Surplus: %d apples", apples_found - apples_needed);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "  ✗ INSUFFICIENT: Not enough apples found");
            RCLCPP_INFO(this->get_logger(), "  - Shortage: %d apples", apples_needed - apples_found);
        }
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr apple_pub_;
    rclcpp::Service<six_ex4::srv::BurrowStatus>::SharedPtr service_;
    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::vector<geometry_msgs::msg::Pose> detected_apples_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotNode>());
    rclcpp::shutdown();
    return 0;
}