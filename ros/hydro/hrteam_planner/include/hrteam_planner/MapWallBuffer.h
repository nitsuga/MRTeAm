/*
 * MapWallBuffer.h
 *
 *  Created on: Jul 23 2011
 *      Author: tuna
 */
#ifndef HRTEAM_MAP_WALL_BUFFER_H_
#define HRTEAM_MAP_WALL_BUFFER_H_

#include "Utils.h"
#include "Position.h"
#include <vector>

namespace hrteam_planner {

  class MapWallBuffer {
  public:
    MapWallBuffer() {}
  MapWallBuffer(double fx, double fy, double sx, double sy): x0(fx), y0(fy), x1(sx), y1(sy){}
    bool operator == (const MapWallBuffer& n) const {
      return ( this->x0 == n.getX0() && 
	       this->y0 == n.getY0() && 
	       this->x1 == n.getX1() && 
	       this->y1 == n.getY1());  
    }
  
    double getX0() const { return x0; }
    double getX1() const { return x1; }
    double getY0() const { return y0; }
    double getY1() const { return y1; }
    
    void setX0(double x) { x0 = x; }
    void setY0(double y) { y0 = y; }
    void setX1(double x) { x1 = x; }
    void setY1(double y) { y1 = y; }
    
  private:
    double x0, y0, x1, y1;
  };

};
#endif
