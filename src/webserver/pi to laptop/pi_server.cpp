// ----- pi_server.cpp -----
// Compile: g++ -std=c++17 pi_server.cpp -o pi_server -lboost_system -lpthread
// Dependencies: sudo apt install libboost-all-dev

#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main() {
    try {
        boost::asio::io_service io;
        // Serial to ESP32
        boost::asio::serial_port serial(io, "/dev/ttyUSB0");
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

        // TCP server
        tcp::acceptor acceptor(io, tcp::endpoint(tcp::v4(), 8000));
        std::cout << "Server listening on port 8000..." << std::endl;

        for (;;) {
            tcp::socket sock(io);
            acceptor.accept(sock);
            std::cout << "Client connected: " << sock.remote_endpoint() << std::endl;

            boost::asio::streambuf buf;
            while (boost::asio::read_until(sock, buf, '\n')) {
                std::istream is(&buf);
                std::string cmd;
                std::getline(is, cmd);
                if (cmd.empty()) continue;
                std::cout << "Forwarding: '" << cmd << "' to ESP32" << std::endl;
                cmd += '\n';
                boost::asio::write(serial, boost::asio::buffer(cmd));
            }
        }
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}