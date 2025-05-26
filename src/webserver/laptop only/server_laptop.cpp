// pi_server.cpp
// Compile with:
//   g++ -std=c++17 pi_server.cpp -o pi_server -lboost_system -lpthread

#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main() {
    try {
        boost::asio::io_context io;

        // 1) Open serial to ESP32
        boost::asio::serial_port serial(io, "/dev/ttyUSB0");
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        std::cout << "[Pi] Serial open on /dev/ttyUSB0 @115200\n";

        // 2) TCP server on port 8000
        tcp::acceptor acceptor(io, tcp::endpoint(tcp::v4(), 8000));
        std::cout << "[Pi] Listening on pi server\n";

        for (;;) {
            tcp::socket sock(io);
            acceptor.accept(sock);
            std::cout << "[Pi] Client connected: " 
                      << sock.remote_endpoint() << "\n";

            boost::asio::streambuf buf;
            while (boost::asio::read_until(sock, buf, '\n')) {
                std::istream is(&buf);
                std::string cmd;
                std::getline(is, cmd);
                if (cmd.empty()) continue;
                std::cout << "[Pi] → ESP32: `" << cmd << "`\n";
                cmd += '\n';
                boost::asio::write(serial, boost::asio::buffer(cmd));
            }
        }
    }
    catch (std::exception &e) {
        std::cerr << "[Pi] Error: " << e.what() << "\n";
    }
    return 0;
}
