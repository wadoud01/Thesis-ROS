#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iostream>
#include <array>
#include <cmath>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class UDPSenderSocket {
public:
    UDPSenderSocket() : sockfd(0) {
        slen = sizeof(si_server);
    }

    void Configure(const char *ip, const char *port) {
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
            perror("UDP SenderSocket - socket creation failed");
            exit(1);
        }

        uint16_t serverPort = static_cast<uint16_t>(std::stoi(port));
        ConfigureSenderSocket(si_server, ip, serverPort);
    }

    int Send(double *dataToSend, std::size_t dataSize) {
        return sendto(sockfd, dataToSend, dataSize, 0, (struct sockaddr *) &si_server, slen);
    }

private:
    bool ConfigureSenderSocket(struct sockaddr_in &si_out, const char *ip, uint16_t port) {
        memset((char *) &si_out, 0, sizeof(si_out));
        si_out.sin_family = AF_INET;
        si_out.sin_port = htons(port);

        if (inet_aton(ip, &si_out.sin_addr) == 0) {
            fprintf(stderr, "inet_aton() failed\n");
            exit(1);
        }
        return true;
    }

    struct sockaddr_in si_server;
    int sockfd;
    socklen_t slen;
};

// Global variables
std::array<double, 8> current_positions;
std::array<double, 8> previous_positions;
bool position_sent = false; // Tracks if the current position has been sent
const double position_threshold = 0.001; // Threshold to detect motion
bool execute_signal = false; // Flag to indicate whether the execute signal is received

ros::Publisher feedback_pub; // Feedback publisher to Unity

// Function to check if the manipulator has stopped moving
bool isStopped() {
    for (size_t i = 0; i < current_positions.size(); ++i) {
        if (std::fabs(current_positions[i] - previous_positions[i]) > position_threshold) {
            return false; // Still moving
        }
    }
    return true; // Stopped
}

// Callback function to monitor joint states
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    for (int i = 0; i < 7; ++i) {
        previous_positions[i] = current_positions[i];
        current_positions[i] = msg->position[i];
    }
    previous_positions[7] = current_positions[7];
    current_positions[7] = msg->position[7] * 2;
}

// Callback for the execute signal
void executeSignalCallback(const std_msgs::Bool::ConstPtr &msg) {
    execute_signal = msg->data;

    if (execute_signal && isStopped() && !position_sent) {
        UDPSenderSocket udpSender;
        udpSender.Configure("130.251.6.21", "1700"); // Update with your server details
        udpSender.Send(current_positions.data(), current_positions.size() * sizeof(double));

        position_sent = true;
        std_msgs::String feedback_msg;
        feedback_msg.data = "Executing to Panda Robot...";
        feedback_pub.publish(feedback_msg);  // Send feedback to Unity
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pos_udp_sender");
    ros::NodeHandle nh;

    feedback_pub = nh.advertise<std_msgs::String>("unity_feedback", 10);  // Feedback topic

    // Subscribe to joint states and execute signal topic
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStateCallback);
    ros::Subscriber execute_signal_sub = nh.subscribe("/execute_signal", 10, executeSignalCallback);

    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();

        // Check if the manipulator has started moving again and reset position_sent flag
        if (!isStopped()) {
            position_sent = false;
        }

        rate.sleep();
    }

    return 0;
}

