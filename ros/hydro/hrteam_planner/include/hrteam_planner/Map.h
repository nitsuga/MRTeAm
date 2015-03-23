/*
 * Map.h
 *
 *  Created on: Dec 21, 2008
 *      Author: richardmarcley
 */

#ifndef HRTEAM_MAP_H_
#define HRTEAM_MAP_H_

#include <string>
#include <vector>
#include <math.h>
#include <stdio.h>

#include "MapMarker.h"
#include "MapWall.h"
#include "MapVirtualWall.h"
#include "MapWallBuffer.h"
#include "Position.h"

namespace hrteam_planner {
  
  class Map {
  public:
    Map();
    Map(int, int, int);
    
    void addMarker(MapMarker marker);
    std::vector<MapMarker> getMarkerById(std::string id);
    MapMarker getMarker(int index);
    std::vector<MapMarker> getMarkers() { return markers; }
    
    /* added for walls */
    void addWall(std::string, int, int, int, int); 
    std::vector<MapWall> getWallById(std::string id);
    MapWall getWall(int index); 
    std::vector<MapWall> getWalls() { return walls; }
    std::vector<MapWallBuffer> getDisplayWallBuffers(){ return displayBuffers; }
    void updateDisplayWallBuffer(MapWall);
    void updateDisplayWallBuffers();
    /* add for walls ends */ 
    
    /* For camera blind spots. Represented as line segments */
    void addVirtualWall(std::string, int, int, int, int);
    
    int getLength() { return length; }
    int getHeight() { return height; }
    int getBufferSize() { return bufferSize; }
    
    bool isWithinBorders( int, int );
    bool isAccessible(int, int);
    bool isPathObstructed( int, int, int, int );
    bool isPathCrossesBuffer(int, int, int, int);        
    bool isPointInBuffer(int, int); 
    
  protected:
    std::vector<MapMarker> markers;
    std::vector<MapWall> walls;
    std::vector<MapVirtualWall> virtualWalls; 
    std::vector<MapWallBuffer> displayBuffers; 
    
    int length;
    int height;  
    int bufferSize;
  };
};

#endif
