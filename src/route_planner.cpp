#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

	start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  
    for (RouteModel::Node* n : current_node->neighbors) {
      	if (n->visited != true) {
        	n->parent = current_node;
           	n->h_value = CalculateHValue(n);
            n->g_value = (current_node->g_value) + (n->distance((*current_node)));
            n->visited = true;
            open_list.push_back(n);
        }
    }
    current_node->visited=true;
}


bool Compare(RouteModel::Node *a, RouteModel::Node *b) {
  float f1 = a->g_value + a->h_value;
  float f2 = b->g_value + b->h_value;
  return f1 > f2; 
}


RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *next_node = nullptr;
    if (open_list.size() > 0) {
        std::sort(open_list.begin(), open_list.end(), Compare);
  	    next_node = open_list.back();
  	    open_list.pop_back();
    }
	return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::vector<RouteModel::Node> resulted_path;

    while (!(current_node == start_node)) {
        path_found.push_back(*current_node);
      	RouteModel::Node *parent = current_node->parent;
    	distance += current_node->distance(*parent);
      	current_node = parent;
    }
    path_found.push_back(*current_node);
  
  	for (int i=path_found.size()-1; i>=0; i--) {
    	resulted_path.push_back(path_found[i]);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return resulted_path;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    while (current_node != nullptr) {
    	if (current_node == end_node) {
        	m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    return;
}
