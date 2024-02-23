#include <asio.hpp>
#include <iostream>
#include <thread>
#include <queue>
#include <chrono>
#include <cstring> // for memcpy
#include <utility> // for std::pair

const int SOURCE_PORT = 1213;
const int DESTINATION_PORT = 1214;
const int PACKET_SIZE = sizeof(float) * 3;
const std::chrono::milliseconds PROCESSING_INTERVAL(50); // 20Hz = every 50ms
const std::chrono::milliseconds DELAY_INTERVAL(500);

std::queue<std::pair<std::chrono::steady_clock::time_point, std::vector<char>>> packet_queue;
bool first_packet_received = false;

void display_data(const std::vector<char>& data, const std::string& prefix) {
    float values[3];
    std::memcpy(values, data.data(), PACKET_SIZE);
    std::cout << prefix << ": [" << values[0] << ", " << values[1] << ", " << values[2] << "]" << std::endl;
}

void process_packets(asio::ip::udp::socket& send_socket, asio::ip::udp::endpoint& send_endpoint) {
    while (true) {
        if (!packet_queue.empty()) {
            auto& front = packet_queue.front();
            auto arrival_time = front.first;
            auto& packet = front.second;
            
            if (std::chrono::steady_clock::now() - arrival_time >= DELAY_INTERVAL) {
                send_socket.send_to(asio::buffer(packet), send_endpoint);
                display_data(packet, "Sent data");
                packet_queue.pop();
            }
        }
        std::this_thread::sleep_for(PROCESSING_INTERVAL);
    }
}

void handle_receive_from(const asio::error_code& error, std::size_t /*bytes_transferred*/, asio::ip::udp::socket& receive_socket, asio::ip::udp::endpoint& sender_endpoint, std::vector<char>& recv_buffer) {
    if (error) {
        std::cerr << "Error receiving data: " << error.message() << std::endl;
        return;
    }

    display_data(recv_buffer, "Received data");
    packet_queue.push({std::chrono::steady_clock::now(), recv_buffer});

    recv_buffer.resize(PACKET_SIZE);
    receive_socket.async_receive_from(asio::buffer(recv_buffer), sender_endpoint,
        [&](const asio::error_code& ec, std::size_t bytes_transferred) {
            handle_receive_from(ec, bytes_transferred, receive_socket, sender_endpoint, recv_buffer);
        }
    );
}

int main() {
    try {
        asio::io_context io_context;

        asio::ip::udp::socket receive_socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), SOURCE_PORT));
        asio::ip::udp::socket send_socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0));

        asio::ip::udp::endpoint sender_endpoint;
        asio::ip::udp::endpoint send_endpoint(asio::ip::udp::v4(), DESTINATION_PORT);

        std::thread packet_processor_thread(process_packets, std::ref(send_socket), std::ref(send_endpoint));

        std::vector<char> recv_buffer(PACKET_SIZE);
        receive_socket.async_receive_from(asio::buffer(recv_buffer), sender_endpoint,
            [&](const asio::error_code& ec, std::size_t bytes_transferred) {
                handle_receive_from(ec, bytes_transferred, receive_socket, sender_endpoint, recv_buffer);
            }
        );

        io_context.run();

        packet_processor_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
