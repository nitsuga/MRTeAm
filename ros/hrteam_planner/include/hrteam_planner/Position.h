/*
 * Position.h
 *
 *  Created on: Dec 26, 2008
 *      Author: richardmarcley
 */
#ifndef HRTEAM_POSITION_H_
#define HRTEAM_POSITION_H_

#include <string>
#include <math.h>
#include "Move.h"

namespace hrteam_planner {

  class Position {
  public:
    Position () {
      Position (0,0,0);
    }
    Position (double x, double y, double theta);
    
    double getX() const;
    void setX(double x);
    double getY() const;
    void setY(double y);
    double getTheta() const;
    void setTheta(double theta);
    
    void add(Position absolutePos);
    void sub(Position absolutePos);
    void rotate(double radians);
    void walk(double distance);
    double getDistance(Position other);
    
    //Do we need this method?  Don't really see the use of it anymore.
    void moveAbsolute (Move absoluteMove);
    
    //transforms the relative move into the absolute coordinates of the
    //new position.
    void moveRelative (Move relativeMove);
    
    std::string* toString();
    
    bool operator==(Position p);
    
  private:
    double x;
    double y;
    double theta;
  };

};
#endif
