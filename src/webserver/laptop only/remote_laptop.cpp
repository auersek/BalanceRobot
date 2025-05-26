// tcp_remote.cpp
// Compile with:
//   g++ -std=c++17 tcp_remote.cpp -o tcp_remote

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

int main() {
    // 1) Connect to “Pi” on localhost:8000
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serv{};
    serv.sin_family = AF_INET;
    serv.sin_port = htons(8000);
    inet_pton(AF_INET, "172.26.65.28", &serv.sin_addr); //172.26.65.28 or 172.26.224.15
    if (connect(sock, (sockaddr*)&serv, sizeof(serv)) < 0) {
        perror("connect");
        return 1;
    }
    std::cout << "[Client] Connected to localhost:8000\n";

    // 2) Raw terminal for single keystrokes
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "Press F/R/C/A/S (Q to quit):\n";
    char ch;
    while (read(STDIN_FILENO, &ch, 1) > 0) {
        if (ch=='Q' || ch=='q') break;
        if (ch=='f'||ch=='r'||ch=='c'||ch=='a'||ch=='s') {
            char buf[2] = { ch, '\n' };
            send(sock, buf, 2, 0);
            std::cout << "[Client] Sent: " << ch << "\n";
        }
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    close(sock);
    return 0;
}
