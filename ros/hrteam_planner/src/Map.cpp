/*
 * Map.cpp
 *
 *  Created on: Dec 21, 2008
 *      Author: richardmarcley
 *    Modified: added walls 
 */

#include "hrteam_planner/Map.h"
#include "hrteam_planner/Utils.h"

#include <iostream>

namespace hrteam_planner {

  Map::Map(){}

  Map::Map(int length, int height, int size) {
    this->length = length;
    this->height = height;
    this->bufferSize = size;
  }

  void Map::addMarker(MapMarker marker)
  {
    markers.push_back(marker);
  }

  std::vector<MapMarker> Map::getMarkerById(std::string id)
  {
    std::vector<MapMarker> foundMarkers;

    for (unsigned int i =0; i< markers.size(); i++) {
      if (markers[i].getId() == id) {
	foundMarkers.push_back(markers[i]);
      }
    }

    return foundMarkers;
  }

  MapMarker Map::getMarker(int index) {
    return markers[index];
  }

  void Map::addWall(std::string lab, int x1, int y1, int x2, int y2){
    MapWall wall(lab, bufferSize, x1, y1, x2, y2);
    walls.push_back(wall);
    updateDisplayWallBuffer(wall);
    // updateWallBuffers();    // this function tests intersecting buffers and removes redundant ones
  }

  void Map::addVirtualWall(std::string lab, int x1, int y1, int x2, int y2){
    MapVirtualWall vwall(lab, bufferSize, x1, y1, x2, y2);
    virtualWalls.push_back(vwall);
    // updateDisplayWallBuffer(wall);
    // updateWallBuffers();    // this function tests intersecting buffers and removes redundant ones
  }

  std::vector<MapWall> Map::getWallById(std::string id){
    std::vector<MapWall> foundWalls;

    for (unsigned int i =0; i< walls.size(); i++) {
      if (walls[i].getId() == id) {
	foundWalls.push_back(walls[i]);
      }
    }

    return foundWalls;
  }

  MapWall Map::getWall(int index){
    return walls[index];
  }

  void Map::updateDisplayWallBuffer(MapWall w){
    //vector<MapWalls>
  }

  void Map::updateDisplayWallBuffers(){
    std::vector<MapWallBuffer> tempBuffers; 
    std::vector<MapWall>::iterator iter; 
    for ( iter = walls.begin(); iter != walls.end(); iter++ ) {
      tempBuffers = iter->getWallBuffers();
      std::vector<MapWallBuffer>::iterator it; 

      // remove all buffers lying outside the map area
      for ( it = tempBuffers.begin(); it != tempBuffers.end(); it++ ){
	if ( !isWithinBorders(it->getX0(), it->getY0()) && !isWithinBorders(it->getX1(), it->getY1()) 
	     && !isPathObstructed(it->getX0(), it->getY0(), it->getX1(), it->getY1()) ){
	  tempBuffers.erase(it);
	  break;
	}
      }
      for ( it = tempBuffers.begin(); it != tempBuffers.end(); it++ ){
	displayBuffers.push_back(*it);
      }
      // 
    }
    displayBuffers = tempBuffers;
  }

  bool Map::isPathObstructed(int x0, int y0, int x1, int y1 ){
    std::vector<MapWall>::iterator iter; 
    for( iter = walls.begin(); iter != walls.end(); iter++ ){
      double ix, iy;
      if ( Utils::get_line_intersection( static_cast<double>(x0), static_cast<double>(y0), static_cast<double>(x1), 
					 static_cast<double>(y1), static_cast<double>(iter->getX0()),
					 static_cast<double>(iter->getY0()),static_cast<double>(iter->getX1()),
					 static_cast<double>(iter->getY1()), &ix, &iy ) )
	{
	  return true;
	}
    }
    return false;
  }

  bool Map::isWithinBorders(int x, int y){
    if ( x > 0 && x < length && y > 0 && y < height ) 
      return true;
    return false; 
  }

  /*! \brief this function returns if a point in the map is accessible. at the moment it only tests 
    if the point is in an open area or on a wall. 
  */
  bool Map::isAccessible(int x, int y){
    std::vector<MapWall>::iterator iter; 
    for( iter = walls.begin(); iter != walls.end(); iter++ ){
      if ( Utils::isPointOnLine( x, y, iter->getX0(), iter->getY0(), iter->getX1(), iter->getY1() , 1) )
	return false; 
    }
    return true; 
  }

  /*!
    \brief returns true if the path from (x0,y0) to (x1,y1) doesn't cross any buffer lines. Doesn't guarantee
    not crossing any walls. e.g. a path is already in the buffer (danger area) and crosses a wall.
	 
    \sa Map::isPathObstructed

  */
  bool Map::isPathCrossesBuffer(int x0, int y0, int x1, int y1){
    std::vector<MapWall>::iterator iter; 
    for( iter = walls.begin(); iter != walls.end(); iter++ ){
      std::vector<MapWallBuffer> buffers = iter->getWallBuffers(); 
      std::vector<MapWallBuffer>::iterator it; 
      for( it = buffers.begin(); it != buffers.end(); it++ ){
	double * ix = new double();
	double * iy = new double();
	if ( Utils::get_line_intersection( static_cast<double>(x0), static_cast<double>(y0), static_cast<double>(x1), 
					   static_cast<double>(y1), static_cast<double>(iter->getX0()),
					   static_cast<double>(iter->getY0()),static_cast<double>(iter->getX1()),
					   static_cast<double>(iter->getY1()), ix, iy ) )
	  return true;
      }
    }
    return false;
  }        


  /*! 
    \brief This function returns true if a point is within a Wall buffer, meaning too close to a wall.
  */
  bool Map::isPointInBuffer(int x, int y){
    std::vector<MapWall>::iterator iter; 
    for( iter = walls.begin(); iter != walls.end(); iter++ ){
      // Regardless of a wall being vertical or horizontal, 
      // P1 has min x val, P2 has max y val, P3 has max x val, P4 has min y val
      if ( x >= iter->getP1x() && 
	   x <= iter->getP3x() && 
	   y <= iter->getP2y() && 
	   y >= iter->getP4y() ) 
	return true; 
    }
    std::vector<MapVirtualWall>::iterator it; 
    for( it = virtualWalls.begin(); it != virtualWalls.end(); it++ ){
      // Regardless of a virtualWall being vertical or horizontal, 
      // P1 has min x val, P2 has max y val, P3 has max x val, P4 has min y val
      if ( x >= it->getP1x() && 
	   x <= it->getP3x() && 
	   y <= it->getP2y() && 
	   y >= it->getP4y() ) 
	return true; 
    }
    return false;
  }

}
