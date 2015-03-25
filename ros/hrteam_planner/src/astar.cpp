#include "hrteam_planner/astar.h"

namespace hrteam_planner {

  double astar::small_cost = 1;
  double astar::diag_cost = sqrt(2 * small_cost * small_cost);
  
  astar::astar(Graph *g)
    : graph(g) {
    this->path.clear();
  }

  astar::astar(Graph g, Node& start, Node& goal)
    : graph(&g) {
    this->path.clear();
    search(start.getID(), goal.getID());
  }

  bool astar::search(int source, int target) {
    start = new _VNode(graph->getNode(source));
    goal  = new _VNode(graph->getNode(target));
    open.push(start);

    while (!open.empty()) {

      _VNode* current = open.top(); open.pop(); // Get and remove the top of the open list

      if(current->id == target) { // Found the path
	construct_path(current);
	return true;
      }

      closed.push_back(current);

      // Add successor nodes of current to the open list
      vector<int> neighbors = graph->getNode(current->id).getNeighbors();
      for (uint i = 0; i < neighbors.size(); i++) {
	_VNode* tmp = new _VNode(graph->getNode(neighbors[i]));
	//double tmpCost = graph->getNode(current->id).getCostTo(tmp->id);
	
	tmp->g = current->g + graph->getNode(current->id).getCostTo(tmp->id);
	tmp->f = tmp->g + octile_h(tmp, goal); // Compute f for this node
	tmp->prev = current;
	
	bool inClosed = false;
	for(uint j = 0; j < closed.size(); j++) {
	  if(closed[j]->id == tmp->id) {
	    inClosed = true;
	    break;
	    if(closed[j]->g > tmp->g) {
	      printf("closed[j].g = %f, tmp.g = %f\n", closed[j]->g, tmp->g);
	      closed[j] = tmp;
	    }
	  }
	}

	if(inClosed || !tmp->accessible) {
	  continue;
	}

	push_update(open, tmp);
      }
    }
    return false;
  }

  double astar::euclidian_h(_VNode* a, _VNode* b) {
    // sqrt(a.x-b.x^2 + a.y-b.y^2)
    return sqrt( (a->x - b->x) * (a->x - b->x) +
		 (a->y - b->y) * (a->y - b->y) );
  }

  double astar::octile_h(_VNode* a, _VNode* b) {
    double dx, dy;
    dx = abs(a->x - b->x);
    dy = abs(a->y - b->y);
    return min(dx,dy) * diag_cost + (max(dx,dy) - min(dx, dy)) * small_cost;
  }

  void astar::push_update(priority_queue<_VNode*, vector<_VNode*>, _Compare> &pq, _VNode* n) {
    list<_VNode*> l; // Hold all node pointers temporarily
    bool found = false;
    
    while(!pq.empty()) {
      _VNode* tmp = pq.top();
      pq.pop();
      if(tmp->id == n->id) {
	found = true;
	if(tmp->g < n->g)
	  pq.push(tmp);
	else
	  pq.push(n);
	break;
      }

      l.push_back(tmp);
    }

    while(!l.empty()) {
      pq.push(l.front());
      l.pop_front();
    }

    if(!found)
      pq.push(n);

    return;
  }

  void astar::construct_path(_VNode* g) {
    path.clear();
    while(g->prev != NULL) {
      path.push_front(g->prev->id);
      g = g->prev;
    }
  }

  /* Returns true if left hand side has a lower f value than right hand side */
  bool astar::_Compare::operator() (_VNode* lhs, _VNode* rhs) {
    // In case of tie breakers best case is a higher g
    /*
      if(lhs->f == rhs->f)
      {
      if(lhs->g > rhs->g)
      return true;
      else
      return false;
      }
      
      return lhs->f > rhs->f;
    */

    // Different operator ordering
    if(lhs->f > rhs->f)
      return true;
    if(lhs->f < rhs->f)
      return false;
    if(lhs->g > rhs->g)
      return true;
    return false;
  }
  
}
