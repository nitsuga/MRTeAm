/*
 * MapVirtualWall.cpp
 *
 *  Created on: Feb 24, 2012
 *      Author: tuna     
 */
#include "hrteam_planner/MapVirtualWall.h"

namespace hrteam_planner {

  MapVirtualWall::MapVirtualWall(std::string i, int size, double fx, double fy, double sx, double sy) 
    : id(i){
    // for now robot size is equal to the buffer size. change it to a function of both size and actuator error
    bufferSize = size * 0.75;
    ( (fx < sx) || (( fx == sx ) && (fy < sy ))) ? 
      assignSortedEndPoints(fx,fy,sx,sy) : assignSortedEndPoints(sx,sy,fx,fy) ;
    initBuffers(); 
  }

  /*!
    Assigns the smaller x valued point as the first. if x values are the same, smaller y valued point becomes the first.
  */
  void MapVirtualWall::assignSortedEndPoints( double xi, double yi, double xj, double yj ) {
    x0 = xi; 
    y0 = yi; 
    x1 = xj;
    y1 = yj;
  }

  void MapVirtualWall::initBuffers(){
    // hack only works for vertical and horizontal walls -- For the demo
    if ( y0 == y1 ){
      p1x = x0 - bufferSize; 
      p1y = y0 + bufferSize; 

      p2x = x1 + bufferSize; 
      p2y = y1 + bufferSize; 

      p3x = x1 + bufferSize; 
      p3y = y1 - bufferSize; 

      p4x = x0 - bufferSize;
      p4y = y0 - bufferSize; 
    }
    else if ( x0 == x1 ) {
      p1x = x0 - bufferSize; 
      p1y = y0 - bufferSize; 

      p2x = x1 - bufferSize; 
      p2y = y1 + bufferSize; 

      p3x = x1 + bufferSize; 
      p3y = y1 + bufferSize; 

      p4x = x0 + bufferSize;
      p4y = y0 - bufferSize; 
    } 
    MapWallBuffer b1(p1x, p1y, p2x, p2y);
    buffers.push_back(b1);
    MapWallBuffer b2(p2x, p2y, p3x, p3y);
    buffers.push_back(b2);
    MapWallBuffer b3(p3x, p3y, p4x, p4y);
    buffers.push_back(b3);
    MapWallBuffer b4(p4x, p4y, p1x, p1y);
    buffers.push_back(b4);
  }


}
