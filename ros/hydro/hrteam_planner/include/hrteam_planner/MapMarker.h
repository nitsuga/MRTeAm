/*
 * MapMarker.h
 *
 *  Created on: Dec 21, 2008
 *      Author: richardmarcley
 */
#ifndef HRTEAM_MAP_MARKER_H_
#define HRTEAM_MAP_MARKER_H_

#include <string>

#include "Utils.h"
#include "Position.h"

namespace hrteam_planner {

  class MapMarker {
  public:
    MapMarker() {}
    //MapMarker(string, double, double);
    //MapMarker(string, double, double, double, double, double, double);  
    
    MapMarker(std::string id, double x, double y, double lx = -1, double ly = -1,
	      double rx = -1, double ry = -1) {
      this->id = id;
      this->x = x;
      this->y = y;
      this->lx = lx; 
      this->ly = ly; 
      this->rx = rx; 
      this->ry = ry;
    }
    
    
    double getBearing(Position position) const;
    
    std::string getId() const {
      return id;
    }
    
    double getX() const {
      return x;
    }
    
    void setX(double x) {
      this->x = x;
    }
    
    double getY() const {
      return y;
    }
    
    void setY(double y) {
      this->y = y;
    }
    
    double getLeftBearing(Position position) const;
    
    double getRightBearing(Position position) const;
    
    double getLeftX() { return lx; }
    double getLeftY() { return ly; }
    
    double getRightX() { return rx; }
    double getRightY() { return ry; }
    
    void print() { 
      std::cout << "Marker " << id 
	   << " left side(" << lx << "," << ly 
	   << ") center(" << x << "," << y 
		<< ") right(" << rx << "," << ry << ")" << std::endl;
    }
    
  private:
    double x, y, lx, ly, rx, ry;
    std::string id;
  };
};
#endif /* MARKER_H_ */
