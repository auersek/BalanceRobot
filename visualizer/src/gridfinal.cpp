#include <SDL2/SDL.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <set>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <algorithm>
#include <thread>
#include "httplib.h" 
#include "json.hpp"
using json = nlohmann::json;

enum class CellState { UNKNOWN, FREE, OBSTACLE, TARGET, ROBOT };

static const int cellSize         = 5;
static const int gridWidth        = 100;
static const int gridHeight       = 100;
static const float physicalCellSize = 0.05f;

float CurXCoord = 0.0f, CurYCoord = 0.0f;
int   prevRobotX = -1, prevRobotY = -1;
std::vector<std::pair<int,int>> path;
size_t pathIndex = 0;
bool   pathActive = false;

const float OBSTACLE_DETECTION_THRESHOLD_MIN_M = 0.05f;
const float OBSTACLE_DETECTION_THRESHOLD_MAX_M = 0.50f;

// occupancy grid + confidence
static std::vector<std::vector<CellState>> grid(
    gridHeight, std::vector<CellState>(gridWidth, CellState::UNKNOWN)
);
static std::vector<std::vector<int>> obstacleConfidence(
    gridHeight, std::vector<int>(gridWidth, 0)
);

// Forward declarations
void sendGoto(const std::string& espIP, int gridX, int gridY);
std::pair<int,int> worldToGrid(float x, float y);
bool fetchPose(const std::string& espIP, int& gridX, int& gridY);
bool fetchLidar(int& lidarSocket,
                float CurXCoord,
                float CurYCoord,
                std::vector<std::vector<CellState>>& grid);
std::vector<std::pair<int,int>> aStarPath(
    int sx, int sy, int gx, int gy,
    const std::vector<std::vector<CellState>>& gridRef
);

// send a UDP command to the ESP
void sendCommand(const std::string& espIP, const std::string& command) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { std::cerr << "Socket creation failed\n"; return; }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port   = htons(5000);
  inet_pton(AF_INET, espIP.c_str(), &addr.sin_addr);
  sendto(sock, command.c_str(), command.size(), 0,
         (sockaddr*)&addr, sizeof(addr));
  close(sock);
}

void sendGoto(const std::string& espIP, int gridX, int gridY) {
  std::ostringstream oss;
  oss << "GOTO " << gridX << " " << gridY;
  sendCommand(espIP, oss.str());
}

bool fetchPose(const std::string& espIP, int& gridX, int& gridY) {
    try {
        httplib::Client cli(espIP.c_str());
        auto res = cli.Get("/pose");
        if (!res || res->status != 200) {
            std::cerr << "HTTP request to /pose failed\n";
            return false;
        }

        auto body = res->body;
        auto doc = json::parse(body);
        gridX = doc["x"];
        gridY = doc["y"];
        std::cerr << "HTTP pose: " << gridX << ", " << gridY << "\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception in fetchPose(): " << e.what() << "\n";
        return false;
    }
}

// world→grid conversion
std::pair<int,int> worldToGrid(float x, float y) {
  int gx = int((x/physicalCellSize) + gridWidth/2.0f);
  int gy = int((y/physicalCellSize) + gridHeight/2.0f);
  gx = std::clamp(gx, 0, gridWidth-1);
  gy = std::clamp(gy, 0, gridHeight-1);
  return {gx, gy};
}

std::pair<float,float> getWorldPointAtDistance(
    float startX, float startY, float angleRad, float dist_m) {
  return { startX + sin(angleRad)*dist_m,
           startY + cos(angleRad)*dist_m };
}

struct Node {
  int x,y; float f,g,h; Node* parent;
  Node(int x_,int y_,float g_,float h_,Node* p=nullptr)
    :x(x_),y(y_),g(g_),h(h_),f(g_+h_),parent(p){}
  bool operator>(Node const& o) const { return f > o.f; }
};

std::vector<std::pair<int,int>> getNeighbors(
    int x,int y,
    const std::vector<std::vector<CellState>>& gridRef) {
  std::vector<std::pair<int,int>> n;
  for(int dx=-1; dx<=1; dx++) for(int dy=-1; dy<=1; dy++){
    if(std::abs(dx)+std::abs(dy)==1){
      int nx=x+dx, ny=y+dy;
      if(nx>=0&&nx<gridWidth&&ny>=0&&ny<gridHeight){
        if(gridRef[ny][nx]==CellState::FREE ||
           gridRef[ny][nx]==CellState::ROBOT)
          n.emplace_back(nx,ny);
      }
    }
  }
  return n;
}

float heuristic(int x1,int y1,int x2,int y2){
  return std::abs(x1-x2) + std::abs(y1-y2);
}

std::vector<std::pair<int,int>> aStarPath(
    int sx,int sy,int gx,int gy,
    const std::vector<std::vector<CellState>>& gridRef) {
  auto hash = [&](int x,int y){ return y*gridWidth + x; };
  std::priority_queue<Node,std::vector<Node>,std::greater<Node>> openSet;
  std::unordered_map<int,Node*> all;
  std::set<std::pair<int,int>> closed;

  Node* start = new Node(sx,sy,0.0f,heuristic(sx,sy,gx,gy));
  openSet.push(*start);
  all[hash(sx,sy)] = start;

  while(!openSet.empty()){
    Node cur = openSet.top(); openSet.pop();
    if(cur.x==gx && cur.y==gy){
      std::vector<std::pair<int,int>> res;
      for(Node* n=&cur; n; n=n->parent)
        res.emplace_back(n->x,n->y);
      std::reverse(res.begin(), res.end());
      for(auto& kv: all) delete kv.second;
      return res;
    }
    closed.insert({cur.x,cur.y});
    for(auto &pr : getNeighbors(cur.x,cur.y,gridRef)){
      int nx=pr.first, ny=pr.second;
      if(closed.count({nx,ny})) continue;
      float tg = cur.g + 1.0f;
      int key = hash(nx,ny);
      if(!all.count(key) || tg < all[key]->g){
        Node* nb = new Node(nx,ny,tg,heuristic(nx,ny,gx,gy), all[hash(cur.x,cur.y)]);
        all[key] = nb;
        openSet.push(*nb);
      }
    }
  }
  for(auto& kv: all) delete kv.second;
  return {};
}

void castRayAndMark(
    float x0,float y0,float x1,float y1,
    float distance,float angleRad,
    std::vector<std::vector<CellState>>& grid) {
  auto [gx0,gy0]     = worldToGrid(x0,y0);
  auto [det_gx,det_gy] = worldToGrid(x1,y1);

  float lim = std::min(distance, OBSTACLE_DETECTION_THRESHOLD_MAX_M);
  auto [fx,fy] = getWorldPointAtDistance(x0,y0,angleRad,lim);
  auto [fgx,fgy] = worldToGrid(fx,fy);

  int cx=gx0, cy=gy0;
  int dx=std::abs(fgx-gx0), dy=std::abs(fgy-gy0);
  int sx=(gx0<fgx?1:-1), sy=(gy0<fgy?1:-1);
  int err=dx-dy;
  while(true){
    if(cx>=0&&cx<gridWidth&&cy>=0&&cy<gridHeight){
      if(grid[cy][cx]==CellState::UNKNOWN)
        grid[cy][cx]=CellState::FREE;
      else if(grid[cy][cx]==CellState::OBSTACLE){
        obstacleConfidence[cy][cx]--;
        if(obstacleConfidence[cy][cx]<4) //how hard obstacle is to remove wall 
          grid[cy][cx]=CellState::FREE;
      }
    }
    if(cx==fgx && cy==fgy) break;
    int e2 = 2*err;
    if(e2 > -dy){ err -= dy; cx += sx; }
    if(e2 <  dx){ err += dx; cy += sy; }
  }

  if(distance >= OBSTACLE_DETECTION_THRESHOLD_MIN_M &&
     distance <= OBSTACLE_DETECTION_THRESHOLD_MAX_M &&
     det_gx>=0&&det_gx<gridWidth&&det_gy>=0&&det_gy<gridHeight &&
     grid[det_gy][det_gx] != CellState::ROBOT) {
    obstacleConfidence[det_gy][det_gx]++;
    if(obstacleConfidence[det_gy][det_gx] >= 3) // number of rays needed to be sure there is an obstacle
      grid[det_gy][det_gx] = CellState::OBSTACLE;
  }
}


bool fetchLidar(int& lidarSocket, float CurXCoord, float CurYCoord, std::vector<std::vector<CellState>>& grid) {

    if (lidarSocket < 0) {
        lidarSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (lidarSocket < 0) {
            perror("socket creation failed");
            return false;
        }
        
        int opt = 1;
        setsockopt(lidarSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));


        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(5005); // Port to receive Lidar data
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(lidarSocket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind failed");
            close(lidarSocket);
            lidarSocket = -1;
            return false;
        }

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(lidarSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::cerr << "Socket initialized on port 5005\n";
    }


    sockaddr_in fromAddr{};
    socklen_t addrLen = sizeof(fromAddr);
    uint8_t buf[64];

    std::cerr << "Waiting for packet...\n";
    int len = recvfrom(lidarSocket, buf, sizeof(buf), 0, (sockaddr*)&fromAddr, &addrLen);

    if (len < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            perror("recvfrom failed");
        }
        return false;
    }

    std::cerr << "Received packet with len = " << len << "\n";
    if (len != 47) {
        std::cerr << "Unexpected packet length: " << len << "\n";
        return false;
    }

    float startAngle = (*(uint16_t*)&buf[4]) / 100.0;
    float stopAngle = (*(uint16_t*)&buf[44]) / 100.0;
    if (stopAngle < startAngle) stopAngle += 360.0;
    float angleStep = (stopAngle - startAngle) / 11.0;

    for (int i = 0; i < 12; ++i) {
        float angleDeg = startAngle + angleStep * i;
        float angleRad = angleDeg * M_PI / 180.0;
        uint16_t dist_mm = *(uint16_t*)&buf[6 + i * 3];
        float dist_m = dist_mm / 1000.0;
        std::cerr << "Angle: " << angleDeg << ", Dist: " << dist_m << "\n";

        float obsX = CurXCoord + sin(angleRad) * dist_m;
        float obsY = CurYCoord + cos(angleRad) * dist_m;
        castRayAndMark(CurXCoord, CurYCoord, obsX, obsY, dist_m, angleRad, grid);

    }

    return true;
}

int main(int argc, char** argv) {

  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window* win = SDL_CreateWindow(
    "Occupancy Grid",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    gridWidth*cellSize, gridHeight*cellSize, 0
  );
  SDL_Renderer* ren = SDL_CreateRenderer(win, -1, 0);


  httplib::Server svr;


  svr.Post("/mark_obstacle", [&](const httplib::Request& req, httplib::Response& res){

    int x = 0, y = 0;
    auto& b = req.body;
    auto comma = b.find(',');
    x = std::stoi(b.substr(1, comma-1));
    y = std::stoi(b.substr(comma+1, b.size()-comma-2));
    x = std::clamp(x, 0, gridWidth-1);
    y = std::clamp(y, 0, gridHeight-1);
    grid[y][x] = CellState::OBSTACLE;
    res.set_header("Access-Control-Allow-Origin","*");
    res.status = 200;
  });

  svr.Get("/pose", [&](const httplib::Request&, httplib::Response& res){
    std::ostringstream os;
    os << "{\"x\":" << prevRobotX
       << ",\"y\":" << prevRobotY
       << ",\"obstacles\":[";
    bool first = true;
    for(int yy=0; yy<gridHeight; yy++){
      for(int xx=0; xx<gridWidth; xx++){
        if(grid[yy][xx]==CellState::OBSTACLE){
          if(!first) os << ",";
          os << "["<<xx<<","<<yy<<"]";
          first = false;
        }
      }
    }
    os << "]}";
    res.set_header("Content-Type","application/json");
    res.set_header("Access-Control-Allow-Origin","*");
    res.set_content(os.str(), "application/json");
  });


  svr.Get("/goto", [&](const httplib::Request& req, httplib::Response& res){
    int ix = std::stoi(req.get_param_value("gx"));
    int iy = std::stoi(req.get_param_value("gy"));
    sendGoto("192.168.4.1", ix, iy);
    res.set_header("Access-Control-Allow-Origin","*");
    res.set_content("OK", "text/plain");
  });


  svr.set_mount_point("/", "./web_root");

  std::thread([&]{ svr.listen("0.0.0.0", 8000); }).detach();

  const std::string espIP = "172.20.10.5"; //ip address of ESP32
  bool running = true;
  int gx, gy;
  static int lidarSocket = -1;

  while(running){
    SDL_Event e;
    while(SDL_PollEvent(&e)){
      if(e.type == SDL_QUIT) running = false;
      if(e.type == SDL_MOUSEBUTTONDOWN){
        int mx = e.button.x / cellSize;
        int my = e.button.y / cellSize;
        auto [sx,sy] = worldToGrid(CurXCoord, CurYCoord);
        path = aStarPath(sx,sy,mx,my,grid);
        pathIndex = 0; pathActive = !path.empty();
        if(pathActive && path.size()>1){
          auto [nx,ny] = path[1];
          sendGoto(espIP, nx, ny);
        }
      }
    }

    if(fetchPose(espIP, gx, gy)){
      gx = std::clamp(gx, 0, gridWidth-1);
      gy = std::clamp(gy, 0, gridHeight-1);
      CurXCoord = (gx - gridWidth/2.0f) * physicalCellSize;
      CurYCoord = (gy - gridHeight/2.0f) * physicalCellSize;
      if(prevRobotX>=0 && prevRobotY>=0 &&
         grid[prevRobotY][prevRobotX]==CellState::ROBOT)
        grid[prevRobotY][prevRobotX] = CellState::FREE;
      if(grid[gy][gx]==CellState::UNKNOWN)
        grid[gy][gx] = CellState::FREE;
      grid[gy][gx] = CellState::ROBOT;
      prevRobotX = gx; prevRobotY = gy;
    }

    if(pathActive && pathIndex < path.size()){
      auto [tgx,tgy] = path[pathIndex];
      auto [cgx,cgy] = worldToGrid(CurXCoord, CurYCoord);
      if(cgx==tgx && cgy==tgy){
        pathIndex++;
        if(pathIndex<path.size()){
          auto [nx,ny] = path[pathIndex];
          sendGoto(espIP, nx, ny);
        } else {
          pathActive = false;
        }
      }
    }


    fetchLidar(lidarSocket, CurXCoord, CurYCoord, grid);

    SDL_SetRenderDrawColor(ren,128,128,128,255);
    SDL_RenderClear(ren);
    for(int y=0; y<gridHeight; y++){
      for(int x=0; x<gridWidth; x++){
        SDL_Rect r{ x*cellSize, y*cellSize, cellSize, cellSize };
        switch(grid[y][x]){
          case CellState::UNKNOWN:  SDL_SetRenderDrawColor(ren,128,128,128,255); break;
          case CellState::FREE:     SDL_SetRenderDrawColor(ren,255,255,255,255); break;
          case CellState::TARGET:   SDL_SetRenderDrawColor(ren,  0,  0,255,255); break;
          case CellState::OBSTACLE: SDL_SetRenderDrawColor(ren,  0,  0,  0,255); break;
          case CellState::ROBOT:    SDL_SetRenderDrawColor(ren,  0,255,  0,255); break;
        }
        SDL_RenderFillRect(ren, &r);
        SDL_SetRenderDrawColor(ren,50,50,50,255);
        SDL_RenderDrawRect(ren, &r);
      }
    }
    SDL_RenderPresent(ren);
    SDL_Delay(16);
  }

  SDL_Quit();
  return 0;
}
