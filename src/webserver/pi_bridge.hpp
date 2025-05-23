// pi_bridge.cpp
// Reads lines from /dev/ttyUSB0 at 115200 baud and sends them to ws://<SERVER_IP>:9002

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>

typedef websocketpp::client<websocketpp::config::asio_client> WSClient;

int main() {
    const std::string uri = "ws://<SERVER_IP>:9002";   // replace <SERVER_IP> with your server’s IP
    const std::string serial_dev = "/dev/ttyUSB0";     // adjust if your ESP32 shows up as ACM0, etc.

    // --- Set up serial port ---
    boost::asio::io_service io;
    boost::asio::serial_port serial(io, serial_dev);
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    // --- Set up WebSocket client ---
    WSClient client;
    client.init_asio(&io);

    websocketpp::lib::error_code ec;
    auto con = client.get_connection(uri, ec);
    if (ec) {
        std::cerr << "WS connection error: " << ec.message() << std::endl;
        return 1;
    }
    client.connect(con);

    // --- Asynchronous serial read loop ---
    boost::asio::streambuf buf;
    std::function<void()> do_read = [&](){
      boost::asio::async_read_until(serial, buf, '\n',
        [&](auto ec2, std::size_t /*len*/){
          if (!ec2) {
            std::string line;
            std::getline(std::istream(&buf), line);
            client.send(con->get_handle(), line, websocketpp::frame::opcode::text);
            do_read();
          }
        });
    };

    // Start reading once WS is open
    client.set_open_handler([&](websocketpp::connection_hdl){
      std::cout << "[bridge] WS connected, starting serial read\n";
      do_read();
    });

    // Run both ASIO loops
    std::thread t_io([&]{ io.run(); });
    client.run();
    t_io.join();

    return 0;
}
