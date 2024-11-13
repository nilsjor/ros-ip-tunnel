#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
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

// Main function to open TUN and print IP payloads
int main() {
    const std::string tunName = "tun0";  // Name of TUN interface
    int tun_fd = createTunInterface(tunName);
    if (tun_fd < 0) {
        return 1;
    }

    const int BUFFER_SIZE = 1500;
    char buffer[BUFFER_SIZE];

    std::cout << "Listening on TUN interface " << tunName << "..." << std::endl;
    
    while (true) {
        // Read data from TUN interface
        int nread = read(tun_fd, buffer, BUFFER_SIZE);
        if (nread < 0) {
            std::cerr << "Error reading from TUN interface" << std::endl;
            break;
        }

        // Print the raw payload (for simplicity, assuming it's text)
        std::cout << "Received packet payload: ";
        for (int i = 0; i < nread; i++) {
            std::cout << buffer[i];
        }
        std::cout << std::endl;
    }

    close(tun_fd);
    return 0;
}
