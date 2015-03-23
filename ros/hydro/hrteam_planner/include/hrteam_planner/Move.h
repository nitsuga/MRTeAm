/*
 * This represents the movement of the robot.
 * It is given as input to the localization module along with observations from the new position.
 * It has similar fields to Position but the coordinates are relative to the robot (or particle).
 * IMPORTANT: since the axis of the robot change during the move, the relative x and y are
 * given relative to the initial set of coordinates.
 */

#ifndef HRTEAM_MOVE_H_
#define HRTEAM_MOVE_H_

#include <iostream>

namespace hrteam_planner {

  class Move {
  public:
    Move() {
      x = 0; y = 0; theta = 0;
    }
    
    Move(double x, double y, double theta);
    
    double getTheta() const {
      return theta;
    }
  
    double getX() const {
      return x;
    }
  
    double getY() const {
      return y;
    }
  
    void print() {
      std::cout << "Move(" << x << "," << y << "," << theta << ")" << std::endl;
    }
    
  private:
    double theta;
    double x;
    double y;
  };

};
#endif
