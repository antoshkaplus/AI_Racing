

#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <array>

#include "ant/grid/grid.hpp"
#include "ant/grid/algorithms.hpp"
#include "ant/geometry/d2.hpp"
#include "ant/graph/graph.hpp"


using namespace model;
using namespace std;


using namespace ant;
using namespace ant::grid;

using namespace ant::geometry::d2::f;

ofstream tiles("./../output/tiles.txt");
ofstream stats("./../output/stats.txt");
ofstream grid("./../output/grid.txt");
ofstream ways("./../output/ways.txt");

double prevSpeed = 0;
bool inited = false;

static Position currentTile(const Car& self) {
    return {static_cast<Int>(self.getY()/800), static_cast<Int>(self.getX()/800)};
}

string tileTypeToString() {
    return "";
}

Grid<TileType> gr;



// UP, DOWN, RIGHT, LEFT
constexpr array<array<bool, 4>, 12> neighbors = {{
    {{false, false, false, false}}, //EMPTY
    {{true, true, false, false}}, //VERTICAL
    {{false,false,true,true}}, //HORIZONTAL
    {{false,true,true,false}}, //LEFT_TOP_CORNER
    {{false,true,false,true}}, //RIGHT_TOP_CORNER
    {{true,false,true,false}}, //LEFT_BOTTOM_CORNER
    {{true,false,false,true}}, //RIGHT_BOTTOM_CORNER
    {{true,true,false,true}}, //RIGHT_HEADED_T
    {{true,true,true,false}}, //LEFT_HEADED_T
    {{true,false,true,true}}, //BOTTOM_HEADED_T 
    {{false,true,true,true}}, //TOP_HEADED_T
    {{true,true,true,true}} //CROSSROADS
}};

vector<Position> waypoints;


void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    if (!inited) {
        inited = true;

        gr = ToGrid(world.getTilesXY());
        gr = gr.Transposed();
        ::grid << gr; 
        for (auto& w : world.getWaypoints()) {
            tiles << "col: " << w[0] << " row: " << w[1] << endl;
            waypoints.emplace_back(w[1], w[0]);  
        }
        auto is_neighbor = [&](const Position& p, grid::Direction d) {
            return neighbors[static_cast<int>(gr[p])][d];
        };
        // shouldn't use grid at all here... or push values inside is_neighbor
        // but there maybe some cool logic 
        BFS<TileType, decltype(is_neighbor)> bfs;
        bfs.Init(gr, is_neighbor);
        
        
        bfs.FindShortestPaths({14,2}, {13,13});
        
        for (int i = 0; i < world.getWaypoints().size(); ++i) {
            int i_next = (i+1) % world.getWaypoints().size();
            auto res = bfs.FindShortestPaths(waypoints[i], waypoints[i_next]);
            ways << waypoints[i] << "; " << waypoints[i_next] << endl;
            for (auto r : res) {
                for (auto p : r) {
                    ways << p << "; ";
                }
                ways << endl;
            }
            ways << endl;
        }
        
    }
    
    
    
    
    
    Position next_tile{self.getNextWaypointY(), self.getNextWaypointX()};
    
    Point target;
    target.x = (next_tile.col + 0.5) * game.getTrackTileSize();
    target.y = (next_tile.row + 0.5) * game.getTrackTileSize();
 //   if (next_tile.ManhattanDistance(currentTile(self)) <= 2) {
    switch (world.getTilesXY()[next_tile.col][next_tile.row]) {
        case LEFT_TOP_CORNER:
            target.x += game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3;
            target.y += game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3; 
//            nextWaypointX += cornerTileOffset;
//            nextWaypointY += cornerTileOffset;
            break;
        case RIGHT_TOP_CORNER:
            target.x -= (game.getTrackTileSize()/2 - game.getTrackTileMargin()  - self.getWidth()/3);
            target.y += game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3; 


//            nextWaypointX -= cornerTileOffset;
//            nextWaypointY += cornerTileOffset;
            break;
        case LEFT_BOTTOM_CORNER:
            target.x += game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3;
            target.y -= (game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3); 

//            nextWaypointX += cornerTileOffset;
//            nextWaypointY -= cornerTileOffset;
            break;
        case RIGHT_BOTTOM_CORNER:
            target.x -= (game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3);
            target.y -= (game.getTrackTileSize()/2 - game.getTrackTileMargin() - self.getWidth()/3); 

//            nextWaypointX -= cornerTileOffset;
//            nextWaypointY -= cornerTileOffset;
            break;
        case RIGHT_HEADED_T:
            break;
        case LEFT_HEADED_T:
            break;
        case TOP_HEADED_T:
            break;
        case BOTTOM_HEADED_T:
            break;
        case CROSSROADS:
            break;
        default:
            break;
    }
 //   }
    double angleToWaypoint = self.getAngleTo(target.x, target.y);
    double speedModule = hypot(self.getSpeedX(), self.getSpeedY());
    
    move.setWheelTurn(angleToWaypoint * 100. / PI);
    move.setEnginePower(1.);
    
    if (speedModule != prevSpeed) {
        stats << speedModule << endl;
    }
    prevSpeed = speedModule;
    if (speedModule * speedModule * abs(angleToWaypoint) > 2.5 * 2.5 * PI || speedModule > 30 ) {
        move.setBrake(true);
    }
}

MyStrategy::MyStrategy() { }
