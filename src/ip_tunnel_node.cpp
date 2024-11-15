#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <cstring>

#include <netinet/ip.h>       // IP header structure
#include <netinet/udp.h>      // UDP header structure
#include <arpa/inet.h>        // Provides inet_ntop for IP address formatting

class IPTunnelNode : public rclcpp::Node {
public:
    IPTunnelNode()
        : Node("ip_tunnel_node"), running_(true) {
        rclcpp::on_shutdown([this]() {
            running_ = false;
        });

        // Declare parameters
        this->declare_parameter<std::string>("pub_topic");
        this->declare_parameter<std::string>("sub_topic");
        this->declare_parameter<std::string>("tun_device", "tun0");

        // QoS parameters
        this->declare_parameter<std::string>("reliability", "reliable");
        this->declare_parameter<std::string>("durability", "volatile");
        this->declare_parameter<int>("history_depth", 10);

        // Get parameters
        this->get_parameter("tun_device", tun_device_);
        if (!this->get_parameter("pub_topic", pub_topic_) ||
            !this->get_parameter("sub_topic", sub_topic_)) {
            RCLCPP_ERROR(this->get_logger(), "Required parameters not set. Please provide names for pub_topic and sub_topic.");
            rclcpp::shutdown();
            return;
        }

        // Open TUN interface
        tun_fd_ = createTunInterface(tun_device_);
        if (tun_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open TUN interface");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Opened TUN interface %s", tun_device_.c_str());

        // Determine logger level (there has to be a better way...)
        debug_ = rcutils_logging_get_logger_effective_level(this->get_logger().get_name()) 
            <= static_cast<RCUTILS_LOG_SEVERITY>(rclcpp::Logger::Level::Debug);

        // Configure QoS
        auto qos = configureQoS();

        // Publisher with configurable QoS
        RCLCPP_INFO(this->get_logger(), "Publishing on topic: %s", pub_topic_.c_str());
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(pub_topic_, qos);

        // Subscription with configurable QoS
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", sub_topic_.c_str());
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            sub_topic_, qos,
            [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
                // Write received data directly to TUN interface
                if (write(tun_fd_, msg->data.data(), msg->data.size()) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to write to TUN interface");
                }
            });

        // Start a separate thread to monitor TUN interface for outgoing packets
        tun_thread_ = std::thread(&IPTunnelNode::monitorTunInterface, this);
    }

    ~IPTunnelNode() {
        running_ = false;
        if (tun_thread_.joinable()) {
            tun_thread_.join();
        }
        if (tun_fd_ >= 0) {
            close(tun_fd_);
        }
    }

private:
    std::string pub_topic_;
    std::string sub_topic_;
    std::string tun_device_;
    bool debug_;
    int tun_fd_;
    std::thread tun_thread_;
    std::atomic<bool> running_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;

    rclcpp::QoS configureQoS() {
        std::string reliability, durability;
        int history_depth;

        this->get_parameter("reliability", reliability);
        this->get_parameter("durability", durability);
        this->get_parameter("history_depth", history_depth);

        rclcpp::QoS qos(history_depth);

        if (reliability == "reliable") {
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        } else if (reliability == "best_effort") {
            qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown reliability, defaulting to Reliable");
            reliability = "reliable";
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        }

        if (durability == "transient_local") {
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        } else if (durability == "volatile") {
            qos.durability(rclcpp::DurabilityPolicy::Volatile);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown durability, defaulting to Volatile");
            durability = "volatile";
            qos.durability(rclcpp::DurabilityPolicy::Volatile);
        }

        RCLCPP_INFO(this->get_logger(), 
            "QoS Settings: '%s', '%s', history_depth=%d", 
            reliability.c_str(), durability.c_str(), history_depth);

        return qos;
    }

    int createTunInterface(const std::string& tun_name) {
        struct ifreq ifr;
        int fd = open("/dev/net/tun", O_RDWR);
        if (fd < 0) {
            std::cerr << "Error: Cannot open TUN device" << std::endl;
            return -1;
        }

        memset(&ifr, 0, sizeof(ifr));
        ifr.ifr_flags = IFF_TUN | IFF_NO_PI;  // TUN mode, no extra packet info
        strncpy(ifr.ifr_name, tun_name.c_str(), IFNAMSIZ);

        if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
            std::cerr << "Error: Could not configure TUN device" << std::endl;
            close(fd);
            return -1;
        }

        return fd;
    }

    void monitorTunInterface() {
        const int BUFFER_SIZE = 1500;
        char buffer[BUFFER_SIZE];

        while (running_) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(tun_fd_, &read_fds);

            // Use select to wait for data on TUN interface
            int ret = select(tun_fd_ + 1, &read_fds, nullptr, nullptr, nullptr);
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error with select on TUN interface");
                break;
            }

            // Read packet from TUN interface
            int nread = read(tun_fd_, buffer, BUFFER_SIZE);
            if (nread < 0) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from TUN interface");
                continue;
            }

            // Publish the data as UInt8MultiArray
            auto message = std_msgs::msg::UInt8MultiArray();
            message.data.resize(nread);
            std::memcpy(message.data.data(), buffer, nread);
            publisher_->publish(message);

            // Print IP header details
            if (debug_) {
                RCLCPP_INFO(this->get_logger(), "Publishing packet from TUN interface, %d bytes", nread);
                struct iphdr* iph = (struct iphdr*)buffer;
                printIpHeader(iph);
                if (iph->protocol == IPPROTO_UDP) {
                    struct udphdr* udph = (struct udphdr*)(buffer + iph->ihl * 4);
                    printUdpHeader(udph);
                }
            }
        }
    }

    // Function to print the IP header details
    void printIpHeader(const struct iphdr* iph) {
        std::ostringstream oss;
        int header_length = iph->ihl * 4;  // IHL field gives length in 4-byte words
        int option_bytes = header_length - 20;

        oss << "IP Header:\n";
        oss << " - Version: " << (int)iph->version << "\n";
        oss << " - Header Length: " << header_length << " bytes (" << option_bytes << " option bytes)\n";
        oss << " - Type of Service: " << (int)iph->tos << "\n";
        oss << " - Total Length: " << ntohs(iph->tot_len) << " bytes\n";
        oss << " - Identification: " << ntohs(iph->id) << "\n";
        oss << " - Flags: " << ((ntohs(iph->frag_off) & 0xE000) >> 13) << "\n";
        oss << " - Fragment Offset: " << (ntohs(iph->frag_off) & 0x1FFF) << "\n";
        oss << " - Time to Live (TTL): " << (int)iph->ttl << "\n";
        oss << " - Protocol: " << (int)iph->protocol << getProtocolName(iph->protocol) << "\n";
        oss << " - Header Checksum: " << ntohs(iph->check) << "\n";

        char src_ip[INET_ADDRSTRLEN];
        char dst_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(iph->saddr), src_ip, INET_ADDRSTRLEN);
        inet_ntop(AF_INET, &(iph->daddr), dst_ip, INET_ADDRSTRLEN);
        oss << " - Source IP: " << src_ip << "\n";
        oss << " - Destination IP: " << dst_ip;

        RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
    }

    std::string getProtocolName(int protocol) {
        switch (protocol) {
            case 1:   return " (ICMP)";
            case 6:   return " (TCP)";
            case 17:  return " (UDP)";
            case 41:  return " (IPv6)";
            case 58:  return " (ICMPv6)";
            default:  return "";
        }
    }

    // Function to print the UDP header details
    void printUdpHeader(const struct udphdr* udph) {
        std::ostringstream oss;
        oss << "UDP Header:\n";
        oss << " - Source Port: " << ntohs(udph->source) << "\n";
        oss << " - Destination Port: " << ntohs(udph->dest) << "\n";
        oss << " - Length: " << ntohs(udph->len) << " bytes\n";
        oss << " - Checksum: " << ntohs(udph->check);

        RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
    }
};

// Main function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IPTunnelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
