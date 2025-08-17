#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

enum ServerState { CONNECTED, DISCONNECTED };

struct Target {
    std::string ip;
    int port;
    size_t expected_bytes;
    int socket_fd;
    sockaddr_in addr;
    ServerState state;
    std::vector<uint8_t> buffer;
};

std::mutex buffer_mutex;
std::vector<uint8_t> combined_buffer;
std::vector<uint8_t> recv_buffer(20);

bool setup_connection(Target& target) {
    target.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (target.socket_fd < 0) return false;

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    setsockopt(target.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    target.addr.sin_family = AF_INET;
    target.addr.sin_port = htons(target.port);
    inet_pton(AF_INET, target.ip.c_str(), &target.addr.sin_addr);

    if (connect(target.socket_fd, (sockaddr*)&target.addr, sizeof(target.addr)) < 0) {
        close(target.socket_fd);
        target.socket_fd = -1;
        target.state = DISCONNECTED;
        return false;
    }

    target.state = CONNECTED;
    return true;
}

bool has_data_error(const std::vector<uint8_t>& buffer, size_t threshold = 40) {
    size_t zero_count = 0;
    for (uint8_t byte : buffer) {
        if (byte == 0) zero_count++;
    }
    return zero_count > threshold;
}

void send_to_serial(const std::vector<uint8_t>& data) {
    std::cout << "[Serial TX] ";
    for (auto byte : data) std::cout << std::hex << (int)byte << " ";
    std::cout << std::endl;
}

void receive_from_serial(std::vector<uint8_t>& buffer) {
    buffer.clear();
    for (int i = 0; i < 20; ++i) buffer.push_back(0xC0 + i); // Dummy data
    std::cout << "[Serial RX] ";
    for (auto byte : buffer) std::cout << std::hex << (int)byte << " ";
    std::cout << std::endl;
}

void tcp_client_thread(std::vector<Target>& targets) {
    enum State { TRANSMIT, RECEIVE };
    State current_state = TRANSMIT;
    int scan_count = 0;
    const std::string ping_msg = "ping";

    const size_t total_bytes = 80 + 52 + 52 + 52;
    combined_buffer.resize(total_bytes, 0);

    while (true) {
        auto cycle_start = std::chrono::high_resolution_clock::now();

        if (current_state == TRANSMIT) {
            std::vector<std::string> diagnostics;
            size_t offset = 0;

            for (auto& target : targets) {
                if (target.state == DISCONNECTED) {
                    if (!setup_connection(target)) {
                        std::fill(combined_buffer.begin() + offset,
                                  combined_buffer.begin() + offset + target.expected_bytes, 0);
                        diagnostics.push_back(target.ip + ": offline");
                        offset += target.expected_bytes;
                        continue;
                    }
                }

                send(target.socket_fd, ping_msg.c_str(), ping_msg.size(), 0);
                std::vector<uint8_t> buffer(target.expected_bytes, 0);
                ssize_t received = recv(target.socket_fd, buffer.data(), buffer.size(), 0);

                if (received > 0) {
                    buffer.resize(target.expected_bytes);
                    target.buffer = buffer;
                    std::copy(buffer.begin(), buffer.end(), combined_buffer.begin() + offset);

                    if (has_data_error(buffer)) {
                        diagnostics.push_back("DATA ERROR from " + target.ip);
                    } else {
                        diagnostics.push_back(target.ip + ": OK");
                    }
                } else {
                    close(target.socket_fd);
                    target.socket_fd = -1;
                    target.state = DISCONNECTED;
                    std::fill(combined_buffer.begin() + offset,
                              combined_buffer.begin() + offset + target.expected_bytes, 0);
                    diagnostics.push_back(target.ip + ": recv error");
                }

                offset += target.expected_bytes;
            }

            {
                std::lock_guard<std::mutex> lock(buffer_mutex);
                send_to_serial(combined_buffer);
            }

            for (const auto& diag : diagnostics) std::cout << "[Diag] " << diag << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            current_state = RECEIVE;
        } else {
            {
                std::lock_guard<std::mutex> lock(buffer_mutex);
                receive_from_serial(recv_buffer);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            current_state = TRANSMIT;
        }

        auto cycle_end = std::chrono::high_resolution_clock::now();
        auto cycle_duration = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start).count();
        std::cout << "[Cycle " << scan_count << "] Duration: " << cycle_duration << " ms\n";
        scan_count++;
    }
}

int main() {
    std::vector<Target> targets = {
        {"192.168.1.91", 12345, 80, -1, {}, DISCONNECTED, {}},
        {"192.168.1.92", 12345, 52, -1, {}, DISCONNECTED, {}},
        {"192.168.1.93", 12345, 52, -1, {}, DISCONNECTED, {}},
        {"192.168.1.94", 12345, 52, -1, {}, DISCONNECTED, {}}
    };

    std::thread tcp_thread(tcp_client_thread, std::ref(targets));
    tcp_thread.join();

    return 0;
}
