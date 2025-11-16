#include <rclcpp/rclcpp.hpp>
#include "six_ex4/srv/burrow_status.hpp"
#include <random>
#include <chrono>
#include <thread>

class BurrowNode : public rclcpp::Node
{
public:
    BurrowNode() : Node("burrow_node"), scenario_count_(0)
    {
        // Create service client
        client_ = this->create_client<six_ex4::srv::BurrowStatus>("check_burrow_status");
        
        RCLCPP_INFO(this->get_logger(), "Burrow node initialized - Service client ready");
        RCLCPP_INFO(this->get_logger(), "Waiting for turtlebot service...");
        
        // Wait for service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Service available! Starting scenarios...\n");
        
        // Run multiple scenarios
        run_scenarios();
    }

private:
    void run_scenarios()
    {
        // Scenario 1: n < s (low resources)
        send_request(2, 10);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Scenario 2: n < s (medium resources)
        send_request(5, 12);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Scenario 3: n = s (full burrow)
        send_request(8, 8);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Scenario 4: n < s (very low resources)
        send_request(1, 15);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Scenario 5: Random scenario keeping n < s
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> size_dist(5, 20);
        
        int s = size_dist(gen);
        std::uniform_int_distribution<> apple_dist(0, s);
        int n = apple_dist(gen);
        
        send_request(n, s);
        
        RCLCPP_INFO(this->get_logger(), "\n========================================");
        RCLCPP_INFO(this->get_logger(), "All scenarios completed!");
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }
    
    void send_request(int current_apples, int burrow_size)
    {
        scenario_count_++;
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "SCENARIO %d", scenario_count_);
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        auto request = std::make_shared<six_ex4::srv::BurrowStatus::Request>();
        request->current_apples = current_apples;
        request->burrow_size = burrow_size;
        
        RCLCPP_INFO(this->get_logger(), "BURROW CLIENT: Preparing service request");
        RCLCPP_INFO(this->get_logger(), "  - Current apples: %d", current_apples);
        RCLCPP_INFO(this->get_logger(), "  - Burrow capacity: %d", burrow_size);
        
        if (current_apples == burrow_size)
        {
            RCLCPP_INFO(this->get_logger(), "  - Status: FULL (n = s)");
        }
        else if (current_apples < burrow_size)
        {
            RCLCPP_INFO(this->get_logger(), "  - Status: NEEDS REFILL (n < s)");
            RCLCPP_INFO(this->get_logger(), "  - Apples needed: %d", burrow_size - current_apples);
        }
        
        RCLCPP_INFO(this->get_logger(), "BURROW CLIENT: Sending request to turtlebot...");
        
        // Send request asynchronously
        auto result = client_->async_send_request(request);
        
        // Wait for response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            
            RCLCPP_INFO(this->get_logger(), "BURROW CLIENT: Response received!");
            RCLCPP_INFO(this->get_logger(), "  - Apples found by turtlebot: %d", response->apples_found);
            RCLCPP_INFO(this->get_logger(), "  - Enough to refill: %s", 
                       response->enough_apples ? "YES" : "NO");
            
            if (current_apples < burrow_size)
            {
                int needed = burrow_size - current_apples;
                if (response->enough_apples)
                {
                    RCLCPP_INFO(this->get_logger(), "  → Burrow can be refilled! (%d needed, %d found)",
                               needed, response->apples_found);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "  → Cannot refill burrow completely (%d needed, %d found)",
                               needed, response->apples_found);
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "  → Burrow already full, no refill needed");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "BURROW CLIENT: Failed to call service");
        }
        
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }
    
    rclcpp::Client<six_ex4::srv::BurrowStatus>::SharedPtr client_;
    int scenario_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BurrowNode>();
    rclcpp::shutdown();
    return 0;
}