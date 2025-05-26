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
    serv.sin_port   = htons(8000);
    inet_pton(AF_INET, "172.26.224.15", &serv.sin_addr); 
    if (connect(sock, (sockaddr*)&serv, sizeof(serv)) < 0) {
        perror("connect");
        return 1;
    }
    std::cout << "[Client] Connected to 172.26.224.15:8000\n";

    // 2) Raw terminal for single keystrokes
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "Press F/R/C/A/S to drive, X to send coordinates, Q to quit.\n";

    char ch;
    while (read(STDIN_FILENO, &ch, 1) > 0) {
        if (ch=='Q' || ch=='q') break;

        // Simple movement commands
        if (ch=='f'||ch=='r'||ch=='c'||ch=='a'||ch=='s') {
            char buf[3] = { static_cast<char>(tolower(ch)), '\n', '\0' };
            send(sock, buf, 2, 0);
            std::cout << "[Client] Sent: " << buf[0] << "\n";
        }
        // Coordinate command
        else if (ch=='x' || ch=='X') {
            // Prompt the user
            std::cout << "Enter X coordinate (integer): ";
            int xval, yval;
            std::cin >> xval;
            std::cout << "Enter Y coordinate (integer): ";
            std::cin >> yval;

            // Format as X<num>Y<num>\n
            char coordBuf[64];
            int len = snprintf(coordBuf, sizeof(coordBuf),
                               "X%dY%d\n", xval, yval);
            if (len > 0 && len < (int)sizeof(coordBuf)) {
                send(sock, coordBuf, len, 0);
                std::cout << "[Client] Sent: " << coordBuf;
            } else {
                std::cerr << "Error formatting coordinate string\n";
            }

            // Clear the '\n' left in stdin after reading yval
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    close(sock);
    return 0;
}
