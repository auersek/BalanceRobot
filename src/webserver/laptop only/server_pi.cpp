#include <iostream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main() {
    try {
        boost::asio::io_context io;

        // --- 1) Wait for the ESP32 to be plugged in ---
        const std::string portPath = "/dev/ttyUSB0";  // or "/dev/ttyACM0", etc.
        std::cout << "[Server] Waiting for ESP32 on " << portPath << " ...\n";
        while (!std::filesystem::exists(portPath)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "[Server] Detected " << portPath << "\n";

        // --- 2) Now open the serial port ---
        boost::asio::serial_port serial(io, portPath);
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        std::cout << "[Server] Opened serial on " << portPath << " @115200\n";

        // --- 3) Start the TCP server as before ---
        tcp::acceptor acceptor(io, tcp::endpoint(tcp::v4(), 8000));
        std::cout << "[Server] Listening on port 8000\n";

        for (;;) {
            tcp::socket sock(io);
            acceptor.accept(sock);
            std::cout << "[Server] Client connected: "
                      << sock.remote_endpoint() << "\n";

            boost::asio::streambuf buf;
            while (boost::asio::read_until(sock, buf, '\n')) {
                std::istream is(&buf);
                std::string cmd;
                std::getline(is, cmd);
                if (cmd.empty()) continue;
                std::cout << "[Server] → ESP32: `" << cmd << "`\n";
                cmd += '\n';
                boost::asio::write(serial, boost::asio::buffer(cmd));
            }
        }
    }
    catch (std::exception &e) {
        std::cerr << "[Server] Error: " << e.what() << "\n";
    }
    return 0;
}
