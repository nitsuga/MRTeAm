#ifndef HRTEAM_ASTAR_H
#define HRTEAM_ASTAR_H

#include "Graph.h"
#include <queue>
#include <list>
#include <cmath>

namespace hrteam_planner {

  class astar {
  public:
    list<int> path;
    
    astar (Graph*);
    astar (Graph, Node&, Node&);
    bool search(int, int); // Search the graph for a path and return true if found
    
    // Wrappers
    bool isPathFound() { return ( !path.empty() || start->id == goal->id ); }
    list<int> getPathToTarget() { return path; }
    
    
  private:
    
    class _Compare; // prototype
    class _VNode;  // prototype
    Graph *graph;
    _VNode *start, *goal;
    vector<_VNode*> closed;
    
    // Private funcs
    double euclidian_h(_VNode*, _VNode*);               // Euclidian Hueristic
    double octile_h(_VNode*, _VNode*);                  // Octile Hueristic
    void push_update(priority_queue<_VNode*, vector<_VNode*>, _Compare>&, _VNode*); // Update open list
    void construct_path(_VNode*); // Constructs a path from arg to start,
    // by tracing backwards until it finds a
    // _VNode with a null 'prev' pointer
    
    // Some statics since we don't want to compute these on the fly
    static double small_cost; // Min c(s,s')
    static double diag_cost;  // sqrt(2*c(s,s')^2)
    
    // Embedded comparator class
    class _Compare 
    {
    public:
      bool operator() (astar::_VNode*, astar::_VNode*); // Returns true if first node has a smaller f value
    };
    
    priority_queue<_VNode*, vector<_VNode*>, _Compare> open;
    
    // Virtual Node Struct: holds astar specific node data
    class _VNode
    {
    public:
      bool accessible;    // coped from the Node
      int id,            // copied from the Node
	x, y;         // copied from the Node
      double g,        // distance from start
	f;       // g + heuristic
      _VNode* prev;  // best predecessor to move from, used to built the path
      
      _VNode(Node &n) // Make a _VNode from a Node
	{
	  this->id = n.getID();
	  this->x = n.getX();
	  this->y = n.getY();
	  this->accessible = n.isAccessible();
	  g = f = 0;
	  prev = NULL;
	}
      
      _VNode(Node n) // Make a _VNode from a Node
	{
	  this->id = n.getID();
	  this->x = n.getX();
	  this->y = n.getY();
	  this->accessible = n.isAccessible();
	  g = f = 0;
	  prev = NULL;
	}
    };
  };
};

#endif
