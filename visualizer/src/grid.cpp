#include <SDL2/SDL.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <cmath>

enum class CellState { UNKNOWN, FREE, OBSTACLE, TARGET, ROBOT };

static const int cellSize   = 25;
static const int gridWidth  = 30;
static const int gridHeight = 30;
static const int physicalCellSize = 0.1;

// Converts a world coordinate (in the same units your robot uses) to a grid cell index
std::pair<int,int> worldToGrid(float x, float y) {
    int gx = static_cast<int>(x / physicalCellSize);
    int gy = static_cast<int>(y / physicalCellSize);
    // clamp to [0..gridWidth-1], [0..gridHeight-1]
    gx = std::max(0, std::min(gridWidth - 1, gx));
    gy = std::max(0, std::min(gridHeight - 1, gy));
    return {gx, gy};
}

// Opens a POSIX serial port and configures it at 115200‐8N1
// Change “portName” to e.g. "/dev/tty.usbserial-0001" or "COM3".
int openSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << ": " << strerror(errno) << "\n";
        return -1;
    }

    // Configure baud rate, 8N1, no flow control
    struct termios tty = {};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr error: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);              // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD);            // no parity
    tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                      // no hardware flow control

    tty.c_iflag &= ~IGNBRK;                       // disable break processing
    tty.c_lflag = 0;                              // no signaling chars, no echo
    tty.c_oflag = 0;                              // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                          // non-blocking read
    tty.c_cc[VTIME] = 1;                          // 0.1 s read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    return fd;
}

int main(int argc, char** argv) {
    // 1) Initialize SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Occupancy Grid",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        gridWidth * cellSize,
        gridHeight * cellSize,
        0
    );
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // 2) Create and initialize the grid
    std::vector<std::vector<CellState>> grid(
        gridHeight, std::vector<CellState>(gridWidth, CellState::UNKNOWN)
    );

    // 3) Open the same serial port your robot uses (e.g. "/dev/tty.usbserial-0001")
    const char* portName = "/dev/tty.usbserial-14210";
    int serialFd = openSerialPort(portName);
    if (serialFd < 0) {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // 4) Main loop: read serial, parse X/Y, update grid, render
    bool running = true;
    SDL_Event event;
    std::string buffer;
    buffer.reserve(64);

    while (running) {
        // — Handle SDL events (window closing, etc.)
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        // — a) Read from serial (non-blocking)
        char readBuf[32];
        int n = read(serialFd, readBuf, sizeof(readBuf));
        if (n > 0) {
            // Append incoming bytes to buffer
            buffer.append(readBuf, n);

            // Process full lines ending in '\n'
            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                // Example incoming: "X:12.34,Y:−5.67"
                float rx = 0.0f, ry = 0.0f;
                if (sscanf(line.c_str(), "X:%f,Y:%f", &rx, &ry) == 2) {
                    // Convert to grid cell
                    auto [gx, gy] = worldToGrid(rx, ry);

                    // Clear previous ROBOT cell(s)
                    for (int yy = 0; yy < gridHeight; ++yy) {
                        for (int xx = 0; xx < gridWidth; ++xx) {
                            if (grid[yy][xx] == CellState::ROBOT) {
                                // Leave it as FREE (assuming visited = free)
                                grid[yy][xx] = CellState::FREE;
                            }
                        }
                    }

                    // If you want to mark UNKNOWN → FREE when visited:
                    if (grid[gy][gx] == CellState::UNKNOWN) {
                        grid[gy][gx] = CellState::FREE;
                    }
                    // Finally, mark the robot’s current cell
                    grid[gy][gx] = CellState::ROBOT;
                }
                // else: malformed line, ignore
            }
        }
        // If n < 0 and errno == EAGAIN, no data available—just continue.

        // — b) Render the grid
        SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255); // background (UNKNOWN)
        SDL_RenderClear(renderer);

        for (int y = 0; y < gridHeight; ++y) {
            for (int x = 0; x < gridWidth; ++x) {
                SDL_Rect rect = {
                    x * cellSize,
                    y * cellSize,
                    cellSize,
                    cellSize
                };

                switch (grid[y][x]) {
                    case CellState::UNKNOWN:
                        SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
                        break;
                    case CellState::FREE:
                        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
                        break;
                    case CellState::OBSTACLE:
                        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                        break;
                    case CellState::TARGET:
                        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
                        break;
                    case CellState::ROBOT:
                        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
                        break;
                }
                SDL_RenderFillRect(renderer, &rect);

                // Draw grid lines in a dark gray
                SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
                SDL_RenderDrawRect(renderer, &rect);
            }
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(16);  // ~60 FPS
    }

    // 5) Clean up
    close(serialFd);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
