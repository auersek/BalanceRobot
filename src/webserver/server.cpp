#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>
#include <set>
using websocketpp::connection_hdl;

int main(){
    typedef websocketpp::server<websocketpp::config::asio> server;
    server ws;
    std::set<connection_hdl,std::owner_less<connection_hdl>> conns;

    ws.init_asio();
    ws.set_open_handler([&](connection_hdl h){ conns.insert(h); });
    ws.set_close_handler([&](connection_hdl h){ conns.erase(h); });
    ws.set_message_handler([&](connection_hdl, auto msg){
        std::cout << "Tilt: " << msg->get_payload() << std::endl;
    });

    ws.listen(9002);
    ws.start_accept();
    std::cout << "[server] Listening on ws://0.0.0.0:9002" << std::endl;
    ws.run();
    return 0;
}
