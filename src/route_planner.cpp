#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find closes nodes to starting and ending coordinates and assign them to RoutePlanner object attributes
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calculate and return distance from passed in node to end_node 
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Find all neighbors of current node
    current_node->FindNeighbors();
    
    // For each newly added neighbor set the parent, the h_value, the g_value, add to open_list, and set visited to true 
    for (RouteModel::Node *new_neighbor: current_node->neighbors) {
        new_neighbor->parent = current_node;
        new_neighbor->h_value = CalculateHValue(new_neighbor);
        new_neighbor->g_value = current_node->g_value + new_neighbor->distance(*current_node);
        open_list.push_back(new_neighbor);
        new_neighbor->visited = true;
    }
}


// Add comparator function to be used with sort
static bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}


RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the list according to f values of each node in the open list
    std::sort(open_list.begin(),open_list.end(),Compare);
    
    // Create new pointer to node with lowest f value, remove node from open list, and return
    RouteModel::Node *next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // Add the current node to the path
    path_found.push_back(*current_node);
    
    // Proceed working through the chain of nodes if the current node is not the start node 
    if (current_node->parent != nullptr) {
        // Create variable to store relevant parent
        RouteModel::Node *node = current_node;
        RouteModel::Node *previous_node = current_node->parent;
        distance += node->distance(*previous_node);
        path_found.push_back(*previous_node);
        // Loop until start node is found
        while (previous_node->parent != nullptr) {
            node = previous_node;
            previous_node = previous_node->parent;
            distance += node->distance(*previous_node);
            path_found.push_back(*previous_node);
        }
    }

    // Change order of vector to start --> finish
    std::reverse(path_found.begin(), path_found.end());
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // Start by adding all neighbors of the start node to the open list
    current_node = start_node;
    open_list.push_back(current_node);
    current_node->visited = true;
   
    // Continue looping until the open list is empty
    while (open_list.size() > 0) {
        // Set the current node after sorting the open list
        current_node = NextNode();
        
        // std::cout << "open_list size is: " << open_list.size() << "\n";
        //
        if (current_node->x == end_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // If we arent done yet expand the search to current nodes neighbors
        AddNeighbors(current_node);
    }

    // We've run out of new nodes to explore and haven't found a path.
    std::cout << "No path found!" << "\n";
}