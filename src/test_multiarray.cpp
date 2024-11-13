#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <cstdlib>
#include <ctime>
#include <vector>

class IPTunnelNode : public rclcpp::Node {
public:
    IPTunnelNode(const std::string &pub_topic, const std::string &sub_topic)
        : Node("ip_tunnel_node") {
        
        // Publisher
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(pub_topic, 10);

        // Subscription
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            sub_topic, 10,
            [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
                std::cout << "Received: [";
                for (size_t i = 0; i < msg->data.size(); ++i) {
                    std::cout << (int)msg->data[i];
                    if (i != msg->data.size() - 1) std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            });

        // Timer to publish random data every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto message = std_msgs::msg::UInt8MultiArray();
                int array_length = rand() % 10 + 1;  // Random length between 1 and 10
                message.data.resize(array_length);

                for (int i = 0; i < array_length; ++i) {
                    message.data[i] = rand() % 256;  // Random byte value (0-255)
                }

                RCLCPP_INFO(this->get_logger(), "Publishing random data array of size %d", array_length);
                publisher_->publish(message);
            });

        std::srand(std::time(0));  // Seed for random number generation
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Error: You must specify both pub_topic and sub_topic names." << std::endl;
        return 1;
    }

    std::string pub_topic = argv[1];
    std::string sub_topic = argv[2];

    auto node = std::make_shared<IPTunnelNode>(pub_topic, sub_topic);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
