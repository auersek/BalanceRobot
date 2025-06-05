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
#include <queue>
#include <unordered_map>
#include <set>

enum class CellState { UNKNOWN, FREE, OBSTACLE, TARGET, ROBOT };

static const int cellSize   = 25;
static const int gridWidth  = 50;
static const int gridHeight = 50;
static const float physicalCellSize = 0.1;

std::vector<std::vector<CellState>> grid(
    gridHeight, std::vector<CellState>(gridWidth, CellState::UNKNOWN)
);

// Converts a world coordinate (in the same units your robot uses) to a grid cell index
std::pair<int,int> worldToGrid(float x, float y) {
    int gx = static_cast<int>((x / physicalCellSize) + 25);
    int gy = static_cast<int>((y / physicalCellSize) + 25);

    std::cout << "X:" << x << std::endl;
    std::cout << "Y:" << y << std::endl;
   
    // clamp to [0..gridWidth-1], [0..gridHeight-1]      
    gx = std::max(0, std::min(gridWidth - 1, gx));
    gy = std::max(0, std::min(gridHeight - 1, gy));

    std::cout << "GX:" << gx << std::endl;
    std::cout << "GY:" << gy << std::endl;
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


struct Node {
    int x, y;
    float f, g, h;
    Node* parent;

    Node(int x_, int y_, float g_, float h_, Node* parent_ = nullptr)
        : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(parent_) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

std::vector<std::pair<int,int>> getNeighbors(int x, int y, const std::vector<std::vector<CellState>>& grid) {
    std::vector<std::pair<int,int>> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (abs(dx) + abs(dy) == 1) {
                int nx = x + dx, ny = y + dy;
                if (nx >= 0 && nx < gridWidth && ny >= 0 && ny < gridHeight && grid[ny][nx] != CellState::OBSTACLE) {
                    neighbors.emplace_back(nx, ny);
                }
            }
        }
    }
    return neighbors;
}
float heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2); // Manhattan distance
}

std::vector<std::pair<int,int>> aStarPath(int sx, int sy, int goalx, int goaly) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<int, Node*> allNodes;
    std::set<std::pair<int,int>> closedSet;

    auto hash = [](int x, int y) { return y * gridWidth + x; };

    Node* start = new Node(sx, sy, 0, heuristic(sx, sy, goalx, goaly));
    openSet.push(*start);
    allNodes[hash(sx, sy)] = start;

    while (!openSet.empty()) {
        Node current = openSet.top(); openSet.pop();
        if (current.x == goalx && current.y == goaly) {
            std::vector<std::pair<int,int>> path;
            for (Node* n = &current; n != nullptr; n = n->parent) {
                path.emplace_back(n->x, n->y);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        closedSet.insert({current.x, current.y});
        for (auto [nx, ny] : getNeighbors(current.x, current.y, grid)) {
            if (closedSet.count({nx, ny})) continue;
            float tentativeG = current.g + 1;
            int hashKey = hash(nx, ny);
            if (!allNodes.count(hashKey) || tentativeG < allNodes[hashKey]->g) {
                Node* neighbor = new Node(nx, ny, tentativeG, heuristic(nx, ny, goalx, goaly), allNodes[hash(current.x, current.y)]);
                allNodes[hashKey] = neighbor;
                openSet.push(*neighbor);
            }
        }
    }
    return {};
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

// Create and initialize the grid


 // Open the same serial port your robot uses (e.g. "/dev/tty.usbserial-0001")
    const char* portName = "/dev/tty.usbserial-14110";
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

    close(serialFd);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}


for (auto& [gx, gy] : path) {
    float wx = gx * physicalCellSize;
    float wy = gy * physicalCellSize;

    // Send next target to ESP32
    std::ostringstream oss;
    oss << "x" << wx << "y" << wy << "\n";
    std::string cmd = oss.str();
    write(serialFd, cmd.c_str(), cmd.length());
    std::cout << "Sent: " << cmd;

    // Wait until robot reaches the target grid cell
    bool reached = false;
    while (!reached && running) {
        char readBuf[32];
        int n = read(serialFd, readBuf, sizeof(readBuf));
        if (n > 0) {
            buffer.append(readBuf, n);

            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                float rx = 0.0f, ry = 0.0f;
                if (sscanf(line.c_str(), "X:%f,Y:%f", &rx, &ry) == 2) {
                    auto [cx, cy] = worldToGrid(rx, ry);
                    CurXCoord = rx; // update global if needed
                    CurYCoord = ry;

                    if (cx == gx && cy == gy) {
                        std::cout << "Reached target (" << gx << "," << gy << ")\n";
                        reached = true;
                        break;
                    }
                }
            }
        }

        SDL_Delay(50); // small sleep to avoid busy waiting
        SDL_PollEvent(&event); // still responsive to SDL quit
        if (event.type == SDL_QUIT) running = false;
    }

    if (!running) break; // early exit if window closed
}






