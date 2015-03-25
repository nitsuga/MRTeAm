#ifndef HRTEAM_EDGE_H
#define HRTEAM_EDGE_H

#include <vector>
#include <iostream>

namespace hrteam_planner {
  class Edge {
  private:
    int from, to;  
    bool usable; 
    
  protected:
    double cost, tempCost;
    
  public:
  Edge(): from (-1), to(-1), cost(0), tempCost(0){ usable = true; } 
  Edge(int n1, int n2, double c = 0): from(n1), to(n2), cost(c), tempCost(0){ usable = true; }
    bool operator == ( Edge e ) {
      int ef = e.getFrom();
      int et = e.getTo(); 
      return ( from == ef && to == et );
    }
    
    double getCost() { return cost + tempCost; }
    void setCost(double c) { cost = c; }
    
    void setTempCost(double tc) { tempCost = tc; }
    double getTempCost() { return tempCost; }
    
    int getFrom(){ return from; }
    int getTo(){ return to; }
    
    void setUsable(bool b) { usable = b; }
    bool isUsable() { return usable; }
    
    void printEdge() const{
      std::cout << "<EDGE-From Node:" << from 
	   << " -To Node:" << to 
	   << " - Cost: " << cost 
		<< " - TempCost: " << tempCost << " >" << std:: endl;
    }
  };
};

#endif
