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
    IPTunnelNode(const std::string &pub_topic, const std::string &sub_topic, const std::string &tun_name)
        : Node("ip_tunnel_node"), running_(true) {

        // Open TUN interface
        tun_fd_ = createTunInterface(tun_name);
        if (tun_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open TUN interface");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Opened TUN interface %s", tun_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing on topic: %s", pub_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", sub_topic.c_str());

        // Publisher for outgoing IP packets
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(pub_topic, 10);

        // Subscription for incoming IP packets
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            sub_topic, 10,
            [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Writing received data to TUN interface");

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
    int tun_fd_;
    std::thread tun_thread_;
    std::atomic<bool> running_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;

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

            RCLCPP_INFO(this->get_logger(), "Publishing packet from TUN interface, %d bytes", nread);
            publisher_->publish(message);

            // Print IP header details
            struct iphdr* iph = (struct iphdr*)buffer;
            printIpHeader(iph);
            if (iph->protocol == IPPROTO_UDP) {
                struct udphdr* udph = (struct udphdr*)(buffer + iph->ihl * 4);
                printUdpHeader(udph);
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
        oss << " - Protocol: " << (int)iph->protocol << "\n";
        oss << " - Header Checksum: " << ntohs(iph->check) << "\n";

        char src_ip[INET_ADDRSTRLEN];
        char dst_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(iph->saddr), src_ip, INET_ADDRSTRLEN);
        inet_ntop(AF_INET, &(iph->daddr), dst_ip, INET_ADDRSTRLEN);
        oss << " - Source IP: " << src_ip << "\n";
        oss << " - Destination IP: " << dst_ip;

        RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
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

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <pub_topic> <sub_topic> [tun_device]" << std::endl;
        return 1;
    }

    std::string pub_topic = argv[1];
    std::string sub_topic = argv[2];
    std::string tun_name = (argc > 3) ? argv[3] : "tun0";

    auto node = std::make_shared<IPTunnelNode>(pub_topic, sub_topic, tun_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
