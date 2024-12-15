#include <iostream>
#include <array>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    // UDP Configuration
    int udp_port = 9999;                // Port for receiving
    struct sockaddr_in receiver_addr, sender_addr;

    int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        perror("Socket creation failed");
        return -1;
    }

    memset(&receiver_addr, 0, sizeof(receiver_addr));
    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_addr.s_addr = INADDR_ANY;
    receiver_addr.sin_port = htons(udp_port);

    if (bind(sockfd, (struct sockaddr *)&receiver_addr, sizeof(receiver_addr)) < 0) {
        perror("Bind failed");
        return -1;
    }

    std::array<double, 7> joint_positions;
    socklen_t addrlen = sizeof(sender_addr);

    std::cout << "Waiting to receive joint positions..." << std::endl;

    while (true) {
        int recvlen = recvfrom(sockfd, joint_positions.data(), joint_positions.size() * sizeof(double), 0, 
                               (struct sockaddr *)&sender_addr, &addrlen);
        if (recvlen > 0) {
            std::cout << "Received joint positions: ";
            for (const auto &pos : joint_positions) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
        } else {
            std::cerr << "Failed to receive data." << std::endl;
        }
    }

    close(sockfd);
    return 0;
}

