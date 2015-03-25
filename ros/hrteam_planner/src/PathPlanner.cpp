/*!
  \file PathPlanner.cpp
  \addtogroup PathPlanner
  @{
*/

#include "hrteam_planner/PathPlanner.h"
#include <limits.h>

#define PATH_DEBUG false

namespace hrteam_planner {

  /*!
    \brief Calculates the shortest path from a start point to a destination point on the navigation graph.

    This function makes necessary calls to populate the PathPlanner::path list, which is a list of node indexes. The list doesn't contain the source (start point) and the target (destination point), only the nodes that the robot needs to get to in sequence, in order to reach its destination.

    First A* algorithm is run on the navigation graph and then the path is smoothed by removing unnecessary waypoints.

    \b Warning a source and a target must be specified prior to this function call.

    \return 0 if path is calculated, 1 if source is not a node or accessible to a valid node, 2 if target is not a node or accessible to a valid node, 3 if no path is found between the source and the target

  */
  int PathPlanner::calcPath(bool cautious){
    const string signature = "PathPlanner::calcPath()> ";
  
    if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
    
      if(PATH_DEBUG) {
	cout << signature << "Source:"; 
	source.printNode(); 
	cout << endl;
	cout << signature << "Target:"; 
	target.printNode();
	cout << endl;
      }

      Node s, t;
      if ( navGraph->isNode(source) ) {
	if(PATH_DEBUG)
	  cout << signature << "Source is a valid Node in the navigation graph" << endl; 
	s = source ;
      }
      else {
	if(PATH_DEBUG)
	  cout << signature << "Source is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
	s = getClosestNode(source, target);
      }
      if ( s.getID() == Node::invalid_node_index )
	return 1;

      if ( navGraph->isNode(target) ) {
	if(PATH_DEBUG)
	  cout << signature << "Target is a valid Node in the navigation graph" << endl; 
	t = target ;
      }
      else {
	if(PATH_DEBUG)
	  cout << signature << "Target is not a valid Node in the navigation graph. Getting closest valid node." << endl; 
	t = getClosestNode(target, source);
      }
      if ( t.getID() == Node::invalid_node_index )
	return 2;

      if(PATH_DEBUG) {
	cout << signature << "s:"; 
	s.printNode(); 
	cout << endl;
	cout << signature << "t:"; 
	t.printNode();
	cout << endl;
      }

      astar newsearch(*navGraph, s, t);
      if ( newsearch.isPathFound() ) {
	path = newsearch.getPathToTarget();
	objectiveSet = false;
	pathCompleted = false;
      
	if(!cautious)
	  smoothPath(path, s, t);
      
	pathCost = calcPathCost(path); 
	pathCalculated = true;
      }
      else {
	return 3;
      }
    }
    return 0;
  }


  bool PathPlanner::isAccessible(Node s, Node t) {
    if ( !navGraph->isNode(s) )
      s = getClosestNode(s, t);

    if ( !navGraph->isNode(t) )
      t = getClosestNode(t, s);

    if ( s.getID() == Node::invalid_node_index || t.getID() == Node::invalid_node_index )
      return false;

    astar newsearch(*navGraph, s, t);
    if ( newsearch.isPathFound() )
      return true;

    return false;
  }


  list<pair<int,int> > PathPlanner::getPathXYBetween(int x1, int y1, int x2, int y2){
  
    // flags show if the (x1, y1) and (x2, y2) are not valid nodes in the navgraph
    bool s_invalid = false; 
    bool t_invalid = false; 

    // flags represent what happens after attempting to get the closest valid node to (x1, y1) and (x2, y2)
    bool no_source = false; 
    bool no_target = false; 

    // create temp nodes for pairs 
    Node temp_s(1, x1, y1); 
    Node temp_t(1, x2, y2);

    /* check if the (x1, y1) is a valid node in the graph, if so get a copy of the node and assign it to s
     * else attempt to get the closest valid node from the graph, if it fails set the no_source flag to true
     * if it succeeds assign the closest valid node to s and set the s_invalid flag to true
     */
    Node s;
    int s_id = navGraph->getNodeID(x1, y1);
    if(s_id == Node::invalid_node_index) {
      s = getClosestNode(temp_s, temp_t);
 
      if(!navGraph->isNode(s))
	no_source = true; 

      s_invalid = true; 
    }
    else {
      s = navGraph->getNode(s_id);
    }

    /* check if the (x2, y2) is a valid node in the graph, if so get a copy of the node and assign it to t
     * else attempt to get the closest valid node from the graph, if it fails set the no_target flag to true
     * if it succeeds assign the closest valid node to t and set the t_invalid flag to true
     */
    Node t;
    int t_id = navGraph->getNodeID(x2, y2);
    if(t_id == Node::invalid_node_index) {
      t = getClosestNode(temp_t, temp_s); 

      if(!navGraph->isNode(t))
	no_target = true; 

      t_invalid = true; 
    }

    // computed path list containing node ids
    list<int> path_c;

    // computed path list containing (x,y) values of the nodes
    list< pair<int,int> > path_c_points;

    /* if either source or the target is invalid, astar can't find a path. 
     * to inform the caller of this function if this situation arises, a special pair <source, target>
     * will be added to the path and the function will return. if any of the values in the pair is -1
     * it will mean the path wasn't found due to invalid source, target or both
     */ 
    if(no_source || no_target) {
      int sval = (no_source) ? -1 : 0; 
      int tval = (no_target) ? -1 : 0;
      pair<int,int> p(sval, tval); 
      path_c_points.push_back(p); 

      return path_c_points;
    }

    // calculate the path. if no path is found return the path_points list empty  
    astar newsearch(*navGraph, s, t);
    if(newsearch.isPathFound()) {
      path_c = newsearch.getPathToTarget();
      smoothPath(path_c, s, t);
    }
    else {
      return path_c_points;
    }
  
    // compute the distance
    if(s_invalid) {
      pair<int, int> st(x1,y1);
      path_c_points.push_back(st);
    }
  
    list<int>::iterator iter;
    for ( iter = path_c.begin(); iter != path_c.end(); iter++ ){
      Node d = navGraph->getNode(*iter);
      pair<int, int> p(d.getX(), d.getY());
      path_c_points.push_back(p);
    }

    if(t_invalid) {
      pair<int, int> tg(x2,y2);
      path_c_points.push_back(tg);
    }

    return path_c_points;
  }


  int PathPlanner::getPathLength(list<pair<int,int> > path){
    double length = 0;
    list<pair<int,int> >::iterator iter, iter_next;
    for ( iter = path.begin(); iter != path.end(); iter++ ){
      iter_next = iter;
      iter_next++;
      if ( iter_next != path.end() ) {
	length += Utils::get_euclidian_distance(iter->first, iter->second, iter_next->first, iter_next->second);
      }
    }
    return static_cast<int>(length);
  }


  double PathPlanner::calcPathCost(list<int> p){
    double pcost = 0;
    list<int>::iterator iter;
    int first;
    Edge * e;

    for( iter = p.begin(); iter != p.end() ; iter++ ){
      first = *iter++;
      if (iter != p.end()){
	e = navGraph->getEdge(first, *iter);
	pcost += e->getCost();
      }
      iter--;
    }
    // add reaching from source and to target costs
    // Note: This will double count when called from estimateCost() and calcPath()

    if ( source.getID() != Node::invalid_node_index && target.getID() != Node::invalid_node_index ){
      if ( !p.empty() ){
	pcost += Utils::get_euclidian_distance(source.getX(), source.getY(),
					       navGraph->getNode(p.front()).getX(),
					       navGraph->getNode(p.front()).getY());
	pcost += Utils::get_euclidian_distance(navGraph->getNode(p.back()).getX(),
					       navGraph->getNode(p.back()).getY(),
					       target.getX(), target.getY());
      }
      else
	pcost += Utils::get_euclidian_distance(source.getX(), source.getY(),
					       target.getX(), target.getY());
    }

    return pcost;
  }

  double PathPlanner::estimateCost(int x1, int y1, int x2, int y2) {
    Node source(1, x1, y1);
    Node target(1, x2, y2);

    return estimateCost(source, target, 1);
  }

  double PathPlanner::estimateCost(Node s, Node t, int l){
    Node sn = getClosestNode(s, t);
    Node tn = getClosestNode(t, s);

    if(tn.getID() < 0 || sn.getID() < 0)
      return INT_MAX;

    double pathCost = Utils::get_euclidian_distance(s.getX(), s.getY(), sn.getX(), sn.getY());
    pathCost += Utils::get_euclidian_distance(t.getX(), t.getY(), tn.getX(), tn.getY());

    astar nsearch(*navGraph, sn, tn);
    if ( nsearch.isPathFound() ){
      list<int> p = nsearch.getPathToTarget();
      pathCost += calcPathCost(p);
    }
    else {
      if(PATH_DEBUG)
	cout << "PathPlanner::estimateCost> no path found to target! "
	     << "Source accessible: " << sn.isAccessible()
	     << ", target accessible: " << tn.isAccessible() << endl;
      pathCost = INT_MAX;
    }

    return pathCost;
  }

  double PathPlanner::getRemainingPathLength(Position src) {
    list<pair<int,int> > path_xy;

    pair<int,int> s(src.getX(), src.getY());
    path_xy.push_back(s); 

    list<int>::iterator iter;
    for(iter = path.begin(); iter != path.end(); iter++) {
      Node d = navGraph->getNode(*iter);
      pair<int, int> p(d.getX(), d.getY());
      path_xy.push_back(p);
    }
  
    pair<int,int> t(target.getX(), target.getY());
    path_xy.push_back(t); 

    return getPathLength(path_xy);
  }


  /*!
    \brief Returns the closest Node in the navigation graph to an arbirary \f$(x, y)\f$ position.

    \param Node \c n, any \f$(x, y)\f$ on the map which we are searching for the closest node
    \param Node \c ref, any \f$(x,y)\f$ on the map, which we are using as a guiding point
    \return Node, this is a member node of the navigation graph, or an invalid node if not found

    This function is used to determine the closest member nodes of the navigation graph to target and the source, 
    since the A* runs only over the member nodes.

    It first asks for the nodes within a region from the navigation graph and returns an accessible node in 
    that area, that has the minimum total distances from the itself to \c n and \c ref nodes. 

    The radius of the search region is initially set to the \c proximity defined by the \c Graph class. 
    If no accessible nodes are found, the radius of the search area is increased and the search is repeated. 
    The maximum search radius is set to 1.5 * proximity. If there are no accessible nodes within that distance
    the robot is probably surrounded by obstacles, therefore this function returns an invalid node.

  */
  Node PathPlanner::getClosestNode(Node n, Node ref){
    const string signature = "PathPlanner::getClosestNode()> ";
  
    Node temp;
    double s_radius = navGraph->getProximity(); 
    double max_radius = navGraph->getProximity() * 1.5; 

    do {
    
      if(PATH_DEBUG)
	cout << signature << "Searching for the closest node within " << s_radius << endl; 

      vector<Node*> nodes = navGraph->getNodesInRegion(n.getX(), n.getY(), s_radius);

      double dist = INT_MAX;

      vector<Node*>::iterator iter;
      for( iter = nodes.begin(); iter != nodes.end(); iter++ ){
	double d = Utils::get_euclidian_distance( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY() );

	if(PATH_DEBUG){
	  cout << "\tChecking ";
	  (*iter)->printNode();
	  cout << endl; 
	  cout << "\tDistance between the n and this node: " << d << endl; 
	}
      
	double d_t = 0.0;
	if(ref.getID() != Node::invalid_node_index)
	  d_t = Utils::get_euclidian_distance((*iter)->getX(), (*iter)->getY(), ref.getX(), ref.getY());

	if(PATH_DEBUG) 
	  cout << "\tDistance between this node to ref: " << d_t << endl; 
      
	if (( d + d_t < dist ) &&
	    !map.isPathObstructed( (*iter)->getX(), (*iter)->getY(), n.getX(), n.getY()) &&
	    (*iter)->isAccessible()) {
	  dist = d + d_t;
	  temp = (*(*iter));
	  if(PATH_DEBUG) {
	    cout << "\tFound a new candidate!: "; 
	    temp.printNode();
	    cout << endl << endl;
	  }
	}
      }
    
      if(temp.getID() == Node::invalid_node_index) {
	s_radius += 0.1 * s_radius; 
	if(PATH_DEBUG) 
	  cout << signature << "Didn't find a suitable candidate. Increasing search radius to: " << s_radius << endl; 
      }

    } while(temp.getID() == Node::invalid_node_index && s_radius <= max_radius); 

    return temp;
  }

  /*!
    \brief Used to remove extra points off of the path
  */
  void PathPlanner::smoothPath(list<int>& pathCalc, Node s, Node t){
    int proximity = navGraph->getProximity();

    if ( pathCalc.size() > 1 ) {
      // smooth the end points
      // if getting to second node from source is shorter and not obstructed remove first node.
      list<int>::iterator iter = pathCalc.begin();
      Node first = navGraph->getNode(*iter++);
      Node second = navGraph->getNode(*iter);
      iter--; // point back to the first element

      if ( PATH_DEBUG ) {
	cout << "source: ";
	s.printNode();
	cout << " - first: " ;
	first.printNode();
	cout << " - second: " ;
	second.printNode();
	cout << endl ;
      }

      if ( !map.isPathObstructed(s.getX(), s.getY(), second.getX(), second.getY()) &&
	   Utils::get_euclidian_distance( s.getX(), s.getY(), second.getX(), second.getY() ) + proximity * 0.5 <
	   ( Utils::get_euclidian_distance( s.getX(), s.getY(), first.getX(), first.getY() ) +
	     Utils::get_euclidian_distance( first.getX(), first.getY(), second.getX(), second.getY() ) )){
	if ( PATH_DEBUG ) cout << "Erasing first" << endl;
	pathCalc.erase(iter);
      }
    }

    if ( pathCalc.size() > 1 ) {
      // if getting to second node from source is shorter and not obstructed remove first node.
      list<int>::iterator iter = pathCalc.end();
      Node last = navGraph->getNode(*(--iter));
      Node onebeforelast = navGraph->getNode(*(--iter));
      iter++;

      if ( PATH_DEBUG ){
	cout << "onebeforelast: ";
	onebeforelast.printNode();
	cout << " - last: " ;
	last.printNode();
	cout << " - target: " ;
	t.printNode();
	cout << endl;
      }

      if ( !map.isPathObstructed(onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY()) &&
	   Utils::get_euclidian_distance( onebeforelast.getX(), onebeforelast.getY(), t.getX(), t.getY() ) + proximity * 0.5 <
	   ( Utils::get_euclidian_distance( onebeforelast.getX(), onebeforelast.getY(), last.getX(), last.getY() ) +
	     Utils::get_euclidian_distance( last.getX(), last.getY(), t.getX(), t.getY() ) )){
	if ( PATH_DEBUG ) cout << "Erasing last" << endl ;
	pathCalc.erase(iter);
      }
    }

    if ( PATH_DEBUG ) {
      cout << "after smoothing: " << endl;
      printPath(pathCalc);
    }
  }

  /*!
    \brief Prints the path node by node
  */
  void PathPlanner::printPath(){
    list<int>::iterator it;
    for ( it = path.begin(); it != path.end(); it++ ){
      navGraph->getNode(*it).printNode() ;
      cout << endl;
    }
  }

  void PathPlanner::printPath(list<int> p){
    list<int>::iterator it ;
    for ( it = p.begin(); it != p.end(); it++ ){
      navGraph->getNode(*it).printNode() ;
      cout << endl;
    }
  }


  void PathPlanner::printPath(list<pair<int,int> > p) {
    list<pair<int,int> >::iterator it ;
    for ( it = p.begin(); it != p.end(); it++ ){
      int nodeId = navGraph->getNodeID(it->first, it->second);
      if ( nodeId == -1 ) {
	cout << "Not a graph node - (" << it->first << ", " << it->second << ")" << endl;
      }
      else {
	navGraph->getNode(nodeId).printNode() ;
	cout << endl;
      }
    }
  }


  bool PathPlanner::allWaypointsValid() {
    list<int>::iterator it;
    for(it = path.begin(); it != path.end(); ++it) {
      if(!navGraph->getNode(*it).isAccessible())
	return false;
    }
    return true;
  }

}
/*! @} */
