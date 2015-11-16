

#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <fstream>

#include "ant/grid.hpp"
#include "ant/geometry/d2.hpp"
#include "ant/graph.hpp"

using namespace model;
using namespace std;


using namespace ant;
using namespace ant::grid;

using namespace ant::geometry::d2::f;

ofstream tiles("./../output/tiles.txt");
ofstream stats("./../output/stats.txt");
ofstream grid("./../output/grid.txt");

double prevSpeed = 0;
bool inited = false;

static Position currentTile(const Car& self) {
    return {static_cast<Int>(self.getY()/800), static_cast<Int>(self.getX()/800)};
}

string tileTypeToString() {
    return "";
}

Grid<TileType> gr;




void MyStrategy::move(const Car& self, const World& world, const Game& game, Move& move) {
    if (!inited) {
        for (auto& w : world.getWaypoints()) {
            tiles << "col: " << w[0] << " row: " << w[1] << endl;  
        }
        inited = true;
        
        
        
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
