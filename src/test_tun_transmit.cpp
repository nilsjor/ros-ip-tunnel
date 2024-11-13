#include <iostream>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <netinet/ip.h>     // IP header structure
#include <netinet/udp.h>    // UDP header structure
#include <arpa/inet.h>      // Provides inet_pton for IP addresses
#include <linux/if.h>
#include <linux/if_tun.h>

// Function to create and configure a TUN interface
int createTunInterface(const std::string& tunName) {
    struct ifreq ifr;
    int fd = open("/dev/net/tun", O_RDWR);
    if (fd < 0) {
        std::cerr << "Error: Cannot open TUN device" << std::endl;
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;  // TUN mode, no extra packet info
    strncpy(ifr.ifr_name, tunName.c_str(), IFNAMSIZ);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        std::cerr << "Error: Could not configure TUN device" << std::endl;
        close(fd);
        return -1;
    }

    std::cout << "TUN interface " << tunName << " created" << std::endl;
    return fd;
}

uint16_t calculateChecksum(void* vdata, size_t length) {
    char* data = (char*)vdata;
    uint32_t acc = 0;

    for (size_t i = 0; i + 1 < length; i += 2) {
        uint16_t word;
        std::memcpy(&word, data + i, 2);
        acc += ntohs(word);
        if (acc > 0xFFFF) {
            acc -= 0xFFFF;
        }
    }

    if (length & 1) {
        uint16_t word = 0;
        std::memcpy(&word, data + length - 1, 1);
        acc += ntohs(word);
        if (acc > 0xFFFF) {
            acc -= 0xFFFF;
        }
    }

    return htons(~acc);
}

void sendTestPacket(int tun_fd) {
    const int packet_size = sizeof(struct iphdr) + sizeof(struct udphdr) + 20;
    char buffer[packet_size];
    memset(buffer, 0, packet_size);

    // IP Header
    struct iphdr* iph = (struct iphdr*)buffer;
    iph->version = 4;
    iph->ihl = 5;
    iph->tos = 0;
    iph->tot_len = htons(packet_size);
    iph->id = htons(12345);
    iph->frag_off = 0;
    iph->ttl = 64;
    iph->protocol = IPPROTO_UDP;
    iph->saddr = inet_addr("10.0.0.2");  // Source IP
    iph->daddr = inet_addr("10.0.0.1");  // Destination IP
    iph->check = calculateChecksum((void*)iph, sizeof(struct iphdr));

    // UDP Header
    struct udphdr* udph = (struct udphdr*)(buffer + sizeof(struct iphdr));
    udph->source = htons(54321);  // Source port
    udph->dest = htons(12345);    // Destination port
    udph->len = htons(sizeof(struct udphdr) + 20);  // UDP header + payload
    udph->check = 0;  // Optional: UDP checksum, often set to 0 in tests

    // Payload
    const char* payload = "Hello from TUN!";
    memcpy(buffer + sizeof(struct iphdr) + sizeof(struct udphdr), payload, strlen(payload));

    std::cout << "Writing test packet to TUN interface..." << std::endl;
    write(tun_fd, buffer, packet_size);
}

int main() {
    const std::string tunName = "tun0";  // Name of TUN interface
    int tun_fd = createTunInterface(tunName);
    if (tun_fd < 0) {
        return 1;
    }

    sendTestPacket(tun_fd);

    close(tun_fd);
    return 0;
}
