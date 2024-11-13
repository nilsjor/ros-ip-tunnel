#include <iostream>
#include <cstring>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <netinet/ip.h>     // IP header structure
#include <netinet/udp.h>    // UDP header structure
#include <linux/if.h>
#include <linux/if_tun.h>
#include <arpa/inet.h>      // Provides inet_ntop

int createTunInterface(const std::string& tunName) {
    struct ifreq ifr;
    int fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        std::cerr << "Error: Cannot open TUN device" << std::endl;
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
    strncpy(ifr.ifr_name, tunName.c_str(), IFNAMSIZ);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        std::cerr << "Error: Could not configure TUN device " << tunName << std::endl;
        close(fd);
        return -1;
    }

    std::cout << "TUN interface " << tunName << " created" << std::endl;
    return fd;
}

void printIpHeader(const struct iphdr* iph) {
    int header_length = iph->ihl * 4;  // IHL field gives length in 4-byte words
    int option_bytes = header_length - 20;
    
    std::cout << "IP Header:" << std::endl;
    std::cout << " - Version: " << (int)iph->version << std::endl;
    std::cout << " - Header Length: " << header_length << " bytes (" << option_bytes << " option bytes)" << std::endl;
    std::cout << " - Type of Service: " << (int)iph->tos << std::endl;
    std::cout << " - Total Length: " << ntohs(iph->tot_len) << " bytes" << std::endl;
    std::cout << " - Identification: " << ntohs(iph->id) << std::endl;
    std::cout << " - Flags: " << ((ntohs(iph->frag_off) & 0xE000) >> 13) << std::endl;
    std::cout << " - Fragment Offset: " << (ntohs(iph->frag_off) & 0x1FFF) << std::endl;
    std::cout << " - Time to Live (TTL): " << (int)iph->ttl << std::endl;
    std::cout << " - Protocol: " << (int)iph->protocol << std::endl;
    std::cout << " - Header Checksum: " << ntohs(iph->check) << std::endl;

    char src_ip[INET_ADDRSTRLEN];
    char dst_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(iph->saddr), src_ip, INET_ADDRSTRLEN);
    inet_ntop(AF_INET, &(iph->daddr), dst_ip, INET_ADDRSTRLEN);
    std::cout << " - Source IP: " << src_ip << std::endl;
    std::cout << " - Destination IP: " << dst_ip << std::endl;
}

void printUdpHeader(const struct udphdr* udph) {
    std::cout << "UDP Header:" << std::endl;
    std::cout << " - Source Port: " << ntohs(udph->source) << std::endl;
    std::cout << " - Destination Port: " << ntohs(udph->dest) << std::endl;
    std::cout << " - Length: " << ntohs(udph->len) << " bytes" << std::endl;
    std::cout << " - Checksum: " << ntohs(udph->check) << std::endl;
}

void printUdpPayload(const char* buffer, int ip_header_len, int udp_len) {
    const char* payload = buffer + ip_header_len + sizeof(struct udphdr);
    int payload_len = udp_len - sizeof(struct udphdr);

    std::cout << "UDP Payload: ";
    for (int i = 0; i < payload_len; ++i) {
        std::cout << payload[i];
    }
    std::cout << std::endl;
}

int main() {
    const std::string tunName = "tun0";
    int tun_fd = createTunInterface(tunName);
    if (tun_fd < 0) {
        return 1;
    }

    const int BUFFER_SIZE = 1500;
    char buffer[BUFFER_SIZE];

    std::cout << "Listening on TUN interface " << tunName << "..." << std::endl;

    while (true) {
        int nread = read(tun_fd, buffer, BUFFER_SIZE);
        if (nread < 0) {
            std::cerr << "Error reading from TUN interface" << std::endl;
            break;
        }

        struct iphdr* iph = (struct iphdr*)buffer;
        printIpHeader(iph);

        if (iph->protocol == IPPROTO_UDP) {
            struct udphdr* udph = (struct udphdr*)(buffer + iph->ihl * 4);
            printUdpHeader(udph);

            // Print the UDP payload (assuming it's text)
            printUdpPayload(buffer, iph->ihl * 4, ntohs(udph->len));
        } else {
            std::cout << "Non-UDP packet received" << std::endl;
        }

        std::cout << std::endl;
    }

    close(tun_fd);
    return 0;
}
