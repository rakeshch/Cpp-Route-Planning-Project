#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// - Using the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(const RouteModel::Node *_node) {
    return _node->distance(*end_node);
}

// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
    // For each node in current_node.neighbors
    for(auto neighbor: current_node->neighbors){
        // set the parent, the h_value, the g_value. 
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);
        // Add the neighbor to open_list
        open_list.emplace_back(neighbor);
        // Set the node's visited attribute to true
        neighbor->visited = true;
    }    
}

// // Compare f values of two nodes and return boolean
// bool Compare(const RouteModel::Node *a, const RouteModel::Node *b){
//     float f1 = a->g_value + a->h_value;
//     float f2 = b->g_value + b->h_value;
//     return f1>f2;
// }

// // Sort the nodes in descending order
// void NodeSort(std::vector<RouteModel::Node *> *node)
// {
//     sort(node->begin(), node->end(), Compare);
// }

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    //NodeSort(&open_list);
    //Using the Lambda funtion for sorting
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b){
        return (a->g_value+a->h_value) > (b->g_value + b->h_value);
    });
    // Create a pointer to the node in the list with the lowest sum.
    auto lowest_f_node = open_list.back();
    // Remove that node from the open_list.
    open_list.pop_back();
    // Return the pointer.
    return lowest_f_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Iterate the current_node (final) through the chain of parents 
    while(current_node->parent != nullptr){
        // std::cout<<current_node->x << " " <<current_node->y <<"\n";
        path_found.emplace_back(*current_node);
        const RouteModel::Node parent = *(current_node->parent);
        // Add the distance from the node to its parent.
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    // Reverse the path so that the start node is the first element of the path_found vector and end node should be the last element
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}

// A* Search algorithm
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    //Start with starting node
    open_list.emplace_back(start_node);
    start_node->visited = true; 

    while(!open_list.empty()){
        // Sort the open_list and return the next node.
        RouteModel::Node *current_node = NextNode();

        //Check if we reached our goal i.e., distance between current node and end node is 0
        //if(current_node->distance(*end_node) == 0){  
        // OR check if current node is end node, that means we reached our goal node          
        if(current_node == end_node){    
            //store final path
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // add all of the neighbors of the current node to the open_list
        AddNeighbors(current_node);
    }
}