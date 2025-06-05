#include <SDL2/SDL.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <set>

enum class CellState { UNKNOWN, FREE, OBSTACLE, TARGET, ROBOT };

static const int cellSize = 25;
static const int gridWidth = 30;
static const int gridHeight = 30;
static const float physicalCellSize = 0.2f;

float CurXCoord = 0.0f, CurYCoord = 0.0f;
int prevRobotX = -1, prevRobotY = -1;

std::vector<std::pair<int,int>> path;
size_t pathIndex = 0;
bool pathActive = false;

std::pair<int,int> worldToGrid(float x, float y) {
    int gx = static_cast<int>((x / physicalCellSize)+15);
    int gy = static_cast<int>((y / physicalCellSize)+15);
    gx = std::max(0, std::min(gridWidth - 1, gx));
    gy = std::max(0, std::min(gridHeight - 1, gy));
    return {gx, gy};
}

int openSerialPort(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios tty = {};
    if (tcgetattr(fd, &tty) != 0) return -1;
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
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
    return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<std::pair<int,int>> aStarPath(int sx, int sy, int gx, int gy, const std::vector<std::vector<CellState>>& grid) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<int, Node*> allNodes;
    std::set<std::pair<int,int>> closedSet;

    auto hash = [](int x, int y) { return y * gridWidth + x; };

    Node* start = new Node(sx, sy, 0, heuristic(sx, sy, gx, gy));
    openSet.push(*start);
    allNodes[hash(sx, sy)] = start;

    while (!openSet.empty()) {
        Node current = openSet.top(); openSet.pop();
        if (current.x == gx && current.y == gy) {
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
                Node* neighbor = new Node(nx, ny, tentativeG, heuristic(nx, ny, gx, gy), allNodes[hash(current.x, current.y)]);
                allNodes[hashKey] = neighbor;
                openSet.push(*neighbor);
            }
        }
    }
    return {};
}

void moveAlongPath(int goalX, int goalY, int serialFd,
                   std::vector<std::vector<CellState>>& grid,
                   float physicalCellSize,
                   float& CurXCoord, float& CurYCoord,
                   std::string& buffer, bool& running) {
    auto [startX, startY] = worldToGrid(CurXCoord, CurYCoord);
    auto path = aStarPath(startX, startY, goalX, goalY, grid);
    if (path.empty()) return;

    for (auto& [goalX, goalY] : path) {
        float wx = goalX * physicalCellSize;
        float wy = goalY * physicalCellSize;

        std::ostringstream oss;
        oss << "x" << wx << "y" << wy << "\n";
        std::string cmd = oss.str();
        write(serialFd, cmd.c_str(), cmd.length());

        bool reached = false;
        while (!reached && running) {
            char readBuf[32];
            int n = read(serialFd, readBuf, sizeof(readBuf));
            if (n > 0) buffer.append(readBuf, n);

            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                float rx = 0.0f, ry = 0.0f;
                if (sscanf(line.c_str(), "X:%f,Y:%f", &rx, &ry) == 2) {
                    CurXCoord = rx;
                    CurYCoord = ry;
                    auto [cx, cy] = worldToGrid(rx, ry);
                    if (cx == goalX && cy == goalY) {
                        std::cout << "Reached" << std::endl;
                        reached = true;
                        break;
                    }
                }
            }
            SDL_Delay(50);
            SDL_Event event;
            SDL_PollEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
                break;
            }
        }
    }
}

int main(int argc, char** argv) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Occupancy Grid", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, gridWidth * cellSize, gridHeight * cellSize, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
    std::vector<std::vector<CellState>> grid(gridHeight, std::vector<CellState>(gridWidth, CellState::UNKNOWN));

    const char* portName = "/dev/tty.usbserial-0001";
    int serialFd = openSerialPort(portName);
    if (serialFd < 0) return -1;

    std::string buffer;
    buffer.reserve(64);

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                int mx = event.button.x / cellSize;
                int my = event.button.y / cellSize;
                auto [startX, startY] = worldToGrid(CurXCoord, CurYCoord);
                path = aStarPath(startX, startY, mx, my, grid);
                pathIndex = 0;
                pathActive = !path.empty();
                // std::cout << pathActive << "YOOOOOOO" << std::endl;
            }

        }

        char readBuf[32];
        int n = read(serialFd, readBuf, sizeof(readBuf));
        if (n > 0) buffer.append(readBuf, n);

        if (pathActive && pathIndex < path.size()) {
            auto [gx, gy] = path[pathIndex];
            auto [cx, cy] = worldToGrid(CurXCoord, CurYCoord);

            if (cx == gx && cy == gy) {
                pathIndex++;
            } else {
            float wx = gx * physicalCellSize;
            float wy = gy * physicalCellSize;
                std::ostringstream oss;
                oss << "x" << wx << "y" << wy << "\\n";
                write(serialFd, oss.str().c_str(), oss.str().length());
                pathActive = true; // stay active
                std::cout << "Sending command: " << oss.str();
            }
}

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string line = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);

            float rx = 0.0f, ry = 0.0f;
            if (sscanf(line.c_str(), "X:%f,Y:%f", &rx, &ry) == 2) {
                auto [gx, gy] = worldToGrid(rx, ry);

                // Mark previous robot cell as FREE
                if (prevRobotX >= 0 && prevRobotY >= 0 && grid[prevRobotY][prevRobotX] == CellState::ROBOT) {
                    grid[prevRobotY][prevRobotX] = CellState::FREE;
                }

                // Update to new position
                if (grid[gy][gx] == CellState::UNKNOWN)
                    grid[gy][gx] = CellState::FREE;
                grid[gy][gx] = CellState::ROBOT;
                prevRobotX = gx;
                prevRobotY = gy;
            }
        }

        SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
        SDL_RenderClear(renderer);
        for (int y = 0; y < gridHeight; ++y) {
            for (int x = 0; x < gridWidth; ++x) {
                SDL_Rect rect = { x * cellSize, y * cellSize, cellSize, cellSize };
                switch (grid[y][x]) {
                    case CellState::UNKNOWN: SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255); break;
                    case CellState::FREE: SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); break;
                    case CellState::OBSTACLE: SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); break;
                    case CellState::TARGET: SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); break;
                    case CellState::ROBOT: SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); break;
                }
                SDL_RenderFillRect(renderer, &rect);
                SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
                SDL_RenderDrawRect(renderer, &rect);
            }
        }
        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    close(serialFd);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
