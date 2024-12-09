#include "route_planner.h"
#include <algorithm>
#include <iostream>

// using std::cout;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    start_node = &m_Model.FindClosestNode(start_x, start_y); 
    end_node   = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
 
    current_node->FindNeighbors();

    for (RouteModel::Node *neighbor : current_node->neighbors){
        if (! neighbor->visited) {
            neighbor->parent  = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->distance(*neighbor) + current_node->g_value;
            neighbor->visited = true;
            // current_node->visited = true;

            open_list.push_back(neighbor);
        }

        // cout << "add neighbors g-value  , distance " <<  current_node->distance(*neighbor) <<"\n" ;
        // cout << "1 add neighbors g-value  , neighbor " << neighbor->g_value << "current node  = " << current_node->g_value <<"\n" ;
    }
    // cout << " Add neighbors -> openlist size = " <<  open_list.size() << "\n";
    
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort( open_list.begin(), open_list.end(), 
    [](const auto &_1st, const auto &_2nd) { // let's fancy using the lambda, [] defines lambda
        return ( (float)(_1st->g_value + _1st->h_value) >  (float)(_2nd->g_value + _2nd->h_value)) ;
    }  
    );

    

    // cout << "NextNode-> openlist size before = " <<  open_list.size() << "\n";
    RouteModel::Node* closestNode = open_list.back(); // get the smallest node
    // auto closestNode = open_list.back(); // get the smallest node
    open_list.pop_back();
    // cout << "NextNode-> openlist size after = " <<  open_list.size() << "\n";

    return closestNode;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node* this_node;

    // int i =0;
    // this_node = current_node;

    // cout << "inside ConstructFinalPath" << "\n";
    // cout <<"this_node " << current_node->parent << "\n";

    // while ( current_node->parent   != nullptr) {

    // we starting from the last node, and each node is linked to its parent node
    // we continue until we reach back to the start node
    while ( current_node   !=  start_node) {
        // cout << "Construction path - self = " << *&current_node  << "\n";
        // if (i++ > 20) break;

        // accumulate the total distance
        distance  += current_node->distance(*current_node->parent ) ;

        // build the path, 
        // since we starting from the bottome and the path needs to be start to end,
        // we insert each new node at the begining
        // alternate would be to append at bottom and then reverse the path

        path_found.insert(path_found.begin(), *current_node);

        // cout <<"inside this_node self = " << *&current_node << ", parent " << current_node->parent << " path size = " << path_found.size() << " g-value = " << current_node->g_value << "\n";
        current_node = current_node->parent;

    }

    // the above loop ends when are at the start node
    // we add start node to the path_found
    path_found.insert(path_found.begin(), *current_node);

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

    // int i = 0;
    // TODO: Implement your solution here.
    // cout << "inside AStarSearch\n" ;


    // We start from teh start node and makes it visited node to True, 
    // else it'll come up again as available node when AddNeighbor is called
    current_node = start_node;
    current_node->visited = true;
    AddNeighbors(current_node);

    while (! open_list.empty()  ) {
        // if (i++>120) {
        //     // ConstructFinalPath(current_node);
        //     break;
        // }
        // cout <<" inside while i = " << i << "\n";
        current_node = NextNode();

        // if (current_node->x == end_node->x &&
        //     current_node->y == end_node->y) 
        // Iterate, until we find the end-node.
        // end-node is calculated at the start and is a valid & existing node  in the map

        if (current_node == end_node ) {
                // cout << "reached the end\n";
                // cout << "found it"<< current_node->x << ", y = "<< current_node->y << " end_node->x = " << end_node->x << " y = " << end_node->y << " \n";
                m_Model.path = ConstructFinalPath(current_node);
                // return;
                break;
            }
        else {
            AddNeighbors(current_node);
            // cout << "continue the search"<< current_node->x << ", y = "<< current_node->y << " end_node->x = " << end_node->x << " y = " << end_node->y << " \n";
        }
    }

    return;
}