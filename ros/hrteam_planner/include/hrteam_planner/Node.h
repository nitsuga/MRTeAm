#ifndef NODE_H
#define NODE_H

#include "Edge.h"
#include <iostream>
#include <vector> 
using namespace std; 

class Node {
protected:

  int id; 

  int x, y; 
  
  bool inBuffer;           // true if within a wall buffer ( too close to a wall ) 

  bool accessible;         // false if the edges of this node are temporarily disabled.

  std::vector<int> neighbors;   // this is the adjacency list of the node. populated from Graph.

  std::vector<hrteam_planner::Edge*> nodeEdges;  // std::vector of edges that goes out from this node ( or comes in if not digraph) 
  
public:

  Node(int i = invalid_node_index, int xt = 0, int yt = 0, bool ib = false)
    : id(i), x(xt), y(yt), inBuffer(ib), accessible(true)
    {} 

  bool operator == (const Node& n) const{
    return ( this->id == n.getID() && this->x == n.getX() && this->y == n.getY() );
  }

  bool operator != (const Node& n) const{
    return ( this->id != n.getID() ); 
  }

  bool operator<(const Node& n) const{
    return ( this->id < n.getID() );
  }

  void setID(int i) { id = i; }

  int getID() const { return id; }

  void setX(int x) { this->x = x; }

  int getX() const { return x; }

  void setY(int y) { this->y = y; }

  int getY() const { return y; }

  bool getInBuffer() const { return inBuffer; }

  void printNode() const{
    cout << "<NODE: " << id << " :(" << x << "," << y << ") >" ;
  }
  
  void printNeighbors() const {
    cout << "Node " << id << " neighbors: " ; 
    for( unsigned int i = 0; i < neighbors.size(); i++ ) {
      cout << neighbors.at(i) << "\t" ; 
    }
    cout << endl ;
  }

  void printNodeEdges() const { 
    cout << "Node " << id << " nodeEdges: " << endl;
    for ( unsigned int i = 0 ; i < nodeEdges.size(); i++ )
      nodeEdges.at(i)->printEdge(); 
  }

  //void setAccessible(bool b) { accessible = b; }
  bool isAccessible() { return accessible; }

  void setAccessible(bool b) {
    accessible = b; 
    std::vector<hrteam_planner::Edge*>::iterator iter; 
    for( iter = nodeEdges.begin(); iter != nodeEdges.end(); iter++ )
      (*iter)->setUsable(b);
  }

  void addNeighbor(int i) { neighbors.push_back(i); }

  std::vector<int> getNeighbors() { return neighbors; }

  void addNodeEdge(hrteam_planner::Edge* e) { nodeEdges.push_back(e); }

  std::vector<hrteam_planner::Edge*>& getNodeEdges() { return nodeEdges; }

  std::vector<hrteam_planner::Edge> getUsableNodeEdges() {
    std::vector<hrteam_planner::Edge> e; 
    std::vector<hrteam_planner::Edge*>::iterator iter;
    for ( iter = nodeEdges.begin() ; iter != nodeEdges.end() ; iter++ ) 
      if ( (*iter)->isUsable() ) 
	e.push_back(*(*iter)); 
    return e; 
  }

  int numNeighbors() const { return neighbors.size(); }

  bool isNeighbor(Node n) {
    std::vector<int>::iterator iter; 
    for( iter = neighbors.begin(); iter != neighbors.end(); iter++ )
      if ( *iter == n.getID() ) 
	return true;
    return false; 
  }

  bool neighborEmpty(){
    return ( neighbors.size() == 0 );
  }

  double getCostTo(int nid)
  {
    double cost;
    std::vector<hrteam_planner::Edge*>::iterator it;
    for(it = nodeEdges.begin(); it != nodeEdges.end(); it++)
    {
      hrteam_planner::Edge* tmp = *it;
      if(tmp->getTo() == this->id && tmp->getFrom() == nid)
      {
        cost = tmp->getCost();
      }
      else if(tmp->getFrom() == this->id && tmp->getTo() == nid)
      {
        cost = tmp->getCost();
      }
    }

    return cost;
  }

  static const int invalid_node_index = -1; 
}; 

#endif

