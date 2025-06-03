// ----- tcp_remote.cpp -----
// Compile: g++ -std=c++17 tcp_remote.cpp -o tcp_remote

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

int main() {
    const char* server_ip = "172.65.";  // Pi address
    const int server_port = 8000;

    // Setup socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(server_port);
    inet_pton(AF_INET, server_ip, &serv_addr.sin_addr);

    if (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("connect"); return 1;
    }
    std::cout << "Connected to Pi at " << server_ip << ":" << server_port << std::endl;

    // Put terminal in raw mode to capture single keystrokes
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "Press F/R/C/A/S to send commands. Press Q to quit." << std::endl;
    char cha;
    while (true) {
        if (read(STDIN_FILENO, &cha, 1) > 0) {
            if (cha == 'q') break;
            if (cha == 'f' || cha == 'r' || cha == 'c' || cha == 'a' || cha == 's') {
                char buf[2] = {cha, '\n'};
                send(sock, buf, 2, 0);
                std::cout << "Sent: " << cha << std::endl;
            }
        }
    }

    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    close(sock);
    return 0;
}
