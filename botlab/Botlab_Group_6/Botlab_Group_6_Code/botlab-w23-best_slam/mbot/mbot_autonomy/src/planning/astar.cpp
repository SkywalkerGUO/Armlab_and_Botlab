#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
#include <iostream>

using namespace std::chrono;

// std::ostream& operator<<(std::ostream& out, const mbot_lcm_msgs::pose_xyt_t& pose){
//     return out << "Pose: " << pose.x << " " << pose.y << " " << pose.theta << std::endl;
// }

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);

    PriorityQueue open; //priority_queue
    std::vector<Node*> closed;

    Node* startNode = new Node(startCell.x, startCell.y);
    startNode -> h_cost = 0;
    startNode -> g_cost = 0;
    startNode -> parent = NULL;
    //startNode -> f_cost = 0;

    Node* goalNode = new Node(goalCell.x, goalCell.y);
    Node* currentNode;

    mbot_lcm_msgs::robot_path_t path;
    path.utime = start.utime;

    if (!distances.isCellInGrid(startCell.x, startCell.y)) {// && distances(start.x, start.y) > params.minDistanceToObstacle) {
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path;
    }
    
    if (!distances.isCellInGrid(goalCell.x, goalCell.y)) {// && distances(goal.x, goal.y) > params.minDistanceToObstacle) {
        path.path.push_back(start); // not sure whether this was supposed to be start or goal
        path.path_length = path.path.size();
        return path;
    }
    
    open.push(startNode);
    
    while(!open.empty()){
        currentNode = open.pop(); //open should be sorted based on the fscore
        //add current to closed list
        closed.push_back(currentNode);
        
        if (currentNode -> cell == goalCell){ //node or cell??
            std::vector<Node*> node_path = extract_node_path(currentNode, startNode);
            path.path = extract_pose_path(node_path, distances);
            path.path_length = path.path.size();
            break;
        }
        //Create each nodes success
        std::vector<Node*>children = expand_node(currentNode, distances, params);

        for(auto &child : children){
            double g_child = g_cost(currentNode, child, distances, params);
            double h_child = h_cost(child, goalNode, distances);

            Node* node = get_from_list(child, closed);
            //if null node in closed list
            if (node){
                if (node -> f_cost() < child -> f_cost()){
                    //we dont need to save this child
                    delete child;
                    continue;
                }
            }
            node = get_from_list(child, open.elements);
            if (node){
                //node in open list
                if (child -> f_cost() < node -> f_cost()){
                    node -> g_cost = g_child;
                    node -> h_cost = h_child;
                    //node -> f_cost = node -> g_cost + node -> f_cost;
                    node -> parent = currentNode;
                }
                continue;
            }
            //Not in open or closed list
            child -> g_cost = g_child;
            child -> h_cost = h_child;
            //child -> f_cost = g_child + h_child;
            child -> parent = currentNode;
            open.push(child);
            
        }
        //add current to closed list
        closed.push_back(currentNode);
    }
    path.path_length = path.path.size();
    //std::cout << path.path;
    return path;

    if (currentNode -> cell != goalNode -> cell){
        //error, no path found
    }

    
    path.path_length = path.path.size();
    return path;
}
double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    //cost to go (heuristic)
    int dx = std::abs(goal->cell.x - from->cell.x);
    int dy = std::abs(goal->cell.y - from->cell.y);
    double diag_distance = 1.414;
    
    double h_cost  = (dx + dy) + (diag_distance - 2) * std::min(dx,dy);
    return h_cost;
}

double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    //cost to come
    int dx = std::abs(from->cell.x - goal->cell.x);
    int dy = std::abs(from->cell.y - goal->cell.y);
    
    float g_cost = from->g_cost; //TODO: Are we saving the g_cost?/
    double diag_distance = 1.414;
    
    if (dx + dy == 2){ //moving diagonally
        g_cost += diag_distance;
    }
    else{ //moving straight
        g_cost += 1;
    }

    //Use obstacle distance to calculate cost from prev to current
    double obst_cost = 0;
    double dist = distances(goal->cell.x, goal->cell.y);
    if (dist > params.minDistanceToObstacle && dist < params.maxDistanceWithCost){
        obst_cost = std::pow(params.maxDistanceWithCost - dist, params.distanceCostExponent);
    }

    g_cost += obst_cost;
    return g_cost;
}

double f_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double f_cost = h_cost(from, goal, distances) + g_cost(from, goal, distances, params);
    return f_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    
    std::vector<Node*> children;
    //TODO: Make the children
    for(int n = 0; n < 8; ++n){
        int cell_x = node->cell.x + xDeltas[n];
        int cell_y = node->cell.y + yDeltas[n];
        Node* childNode = new Node(cell_x, cell_y);

        if(!distances.isCellInGrid(cell_x, cell_y))
            continue;
            
        if(distances(cell_x, cell_y) <= params.minDistanceToObstacle)
            continue;
        
        childNode -> parent = node; // set the current node as the parent for each child
        children.push_back(childNode); // add the child to the vector of children
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    //We need to backtrack the path from the goal node taking min distances back to start node
    Node* curr_node = goal_node;

    while (curr_node -> parent != NULL){ // while the parent is not null (top of tree)
        path.push_back(curr_node); // add current node to path vector
        curr_node = curr_node -> parent; // now the current node is the parent
    }
    path.push_back(curr_node); // add the last (top) node that has null parent
    // if (curr_node -> cell != start_node -> cell){
    //     std::cout << "path doesn't begin with start" << std::endl;
    // }

    //Now path is from goal to start, we can just reverse it
    std::reverse(path.begin(), path.end());

    // std::cout << "path length: " << path.size() << std::endl;

    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    for (size_t i = 0; i < nodes.size(); ++i){
        Point<double> p = grid_position_to_global_position(nodes[i] -> cell, distances);
        mbot_lcm_msgs::pose_xyt_t pose;
        pose.x = p.x;
        pose.y = p.y;
        pose.theta = 0;
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
