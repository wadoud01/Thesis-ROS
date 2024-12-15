#include <iostream>
#include <array>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>

// Global variables
std::mutex mtx;
bool pos_received = false; // Tracks whether a position was received
std::array<double, 7> joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Function for the UDP receiver
void udpReceiver(int port) {
    // UDP CONNECTION SETUP
    struct sockaddr_in receiver_addr, sender_addr;
    int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        perror("Socket creation failed");
        exit(1);
    }

    memset(&receiver_addr, 0, sizeof(receiver_addr));
    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all network interfaces
    receiver_addr.sin_port = htons(port);      // Port for receiving

    if (bind(sockfd, (struct sockaddr *)&receiver_addr, sizeof(receiver_addr)) < 0) {
        perror("Bind failed");
        exit(1);
    }

    int recvlen;
    int bufSize = 2048;
    unsigned char buf[bufSize];
    socklen_t addrlen = sizeof(sender_addr);

    std::cout << "Waiting to receive joint positions..." << std::endl;

    while (true) { // Infinite loop
        recvlen = recvfrom(sockfd, buf, bufSize, 0, (struct sockaddr *)&sender_addr, &addrlen);

        if (recvlen > 0) {
            buf[recvlen] = 0; // Null-terminate the received buffer
            double *bufDouble = reinterpret_cast<double *>(buf);

            // CRITICAL SECTION: Lock mutex to safely update shared data
            mtx.lock();
            for (int i = 0; i < 7; i++) {
                joint_positions[i] = bufDouble[i];
            }
            pos_received = true;
            mtx.unlock();

            // Display the received positions
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
}

int main() {
    int udp_port = 1700; // Update this to match the sender's port
    udpReceiver(udp_port);
    return 0;
}

