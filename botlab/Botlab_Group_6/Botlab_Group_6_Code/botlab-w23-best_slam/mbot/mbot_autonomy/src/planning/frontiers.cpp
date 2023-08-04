#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/timestamp.h>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>
#include <algorithm>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
mbot_lcm_msgs::robot_path_t path_to_frontier(const frontier_t& frontier,
                                              const mbot_lcm_msgs::pose_xyt_t& pose,
                                              const OccupancyGrid& map,
                                              const MotionPlanner& planner);
// mbot_lcm_msgs::pose_xyt_t nearest_navigable_cell(mbot_lcm_msgs::pose_xyt_t pose,
//                                                   Point<float> desiredPosition,
//                                                  mamaaanner);
mbot_lcm_msgs::pose_xyt_t search_to_nearest_free_space(Point<float> position,
                                                        const OccupancyGrid& map,
                                                        const MotionPlanner& planner);
double path_length(const mbot_lcm_msgs::robot_path_t& path);

mbot_lcm_msgs::robot_path_t nearest_navigable_cell(mbot_lcm_msgs::pose_xyt_t pose,
                                                  Point<float> desiredPosition,
//                                                  mbot_lcm_msgs::robot_path_t& path,
                                                  const OccupancyGrid& map,
                                                  const MotionPlanner& planner)
{
    mbot_lcm_msgs::pose_xyt_t goalPose;
    goalPose.x = desiredPosition.x;
    goalPose.y = desiredPosition.y;

    std::queue<Point<int>> q;
    std::vector<Point<int>> visited;

    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    q.push(global_position_to_grid_cell(Point<float>(desiredPosition.x, desiredPosition.y), map));
    visited.push_back(global_position_to_grid_cell(desiredPosition, map));

    while (!q.empty()) {
        cell_t curr = q.front();
        q.pop();

        Point<float> curr_point = grid_position_to_global_position(curr, map);
        mbot_lcm_msgs::pose_xyt_t curr_pose;
        curr_pose.x = curr_point.x;
        curr_pose.y = curr_point.y;
        if (planner.isValidGoal(curr_pose))
        {
            mbot_lcm_msgs::robot_path_t curr_path = planner.planPath(pose, curr_pose);
            if (true)//(planner.isPathSafe(curr_path))
            { 
                //path = curr_path;
                std::cout << "Found Path: " << curr_pose.x << ", " << curr_pose.y << std::endl;
                return curr_path;
            }
            else{
                std::cout << "path is not safe \n";
            }
        }

        //add neighbors to queue
        for (int n = 0; n < 8; ++n)
        {
           Point<int> curr_cell2(curr.x + xDeltas[n], curr.y + yDeltas[n]);
           bool is_visited = std::find(visited.begin(), visited.end(), curr_cell2) != visited.end();
            if (!is_visited) {
                q.push(curr_cell2);
                visited.push_back(curr_cell2);
            }
        }
    }
}


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map, 
                                           const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;
    
    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);
  
    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            
            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);
                
                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }    
    return frontiers;
}

struct CompareCentroids
{
    CompareCentroids(mbot_lcm_msgs::pose_xyt_t robotPose) { this->robotPose = robotPose;}
    inline bool operator() (const Point<double>& centr_1, const Point<double>& centr_2)
    {
        // Diff 1
        float diff_1_x = robotPose.x - centr_1.x;
        float diff_1_y = robotPose.y - centr_1.y;
        float diff_1 = diff_1_x * diff_1_x + diff_1_y * diff_1_y;
        // Diff 2
        float diff_2_x = robotPose.x - centr_2.x;
        float diff_2_y = robotPose.y - centr_2.y;
        float diff_2 = diff_2_x * diff_2_x + diff_2_y * diff_2_y;

        return (diff_1 < diff_2);
    }
    mbot_lcm_msgs::pose_xyt_t robotPose;
};

frontier_processing_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers, 
                                            const mbot_lcm_msgs::pose_xyt_t& robotPose,
                                            const OccupancyGrid& map,
                                            const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */

    // First, choose the frontier to go to
    // Initial alg: find the nearest one

    // Returnable path
    //CHANCE OF RETURNING HOME
    mbot_lcm_msgs::robot_path_t path;
    path.utime = utime_now();
    path.path_length = 1;
    path.path.push_back(robotPose);
    int unreachable_frontiers = 0;
    Point<double> mid_point;
    Point<double> nearest_point;
    std::vector<Point<float>> frontier_mid;
    CompareCentroids compare(robotPose);
    mbot_lcm_msgs::pose_xyt_t end_pose;
    
    //find the nearest one first, check if it's valid. if not vaild, go to the center of the frontiers.
    for (auto &frontier : frontiers){
        mid_point = find_frontier_centroid(frontier);
        frontier_mid.push_back(mid_point);
    }

    //nearest_point = frontier_mid[0];
    for (int i = 0; i<frontier_mid.size(); i++) {
        nearest_point = frontier_mid[i];
        bool flag = compare(nearest_point,frontier_mid[i]); //nearest< current ,return 1
        if (!flag) nearest_point = frontier_mid[i]; // if nearest point is not closer, then reset nearest point to the current frontier
        else continue;
    }
    end_pose.x = nearest_point.x;
    end_pose.y = nearest_point.y;
    end_pose.theta = robotPose.theta;
    end_pose.utime = utime_now();
    path = planner.planPath(robotPose, end_pose);
    Point<float> desiredPosition;
    desiredPosition.x = end_pose.x;
    desiredPosition.y = end_pose.y;
    path = nearest_navigable_cell(robotPose,desiredPosition,map,planner); 

    return frontier_processing_t(path, unreachable_frontiers);
}

bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell is a frontier if it has log-odds 0 and a neighbor has log-odds < 0
    
    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }
    
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };
    
    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }
    
    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);
    
    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };
 
    frontier_t frontier;
    
    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();
        
        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));
        
        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end()) 
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    
    return frontier;
}

Point<double> find_frontier_centroid(const frontier_t& frontier)
{
    // Using the mid point of the frontier
    Point<double> mid_point;
    int index = (int)(frontier.cells.size() / 2.0);
    // printf("index: %d, size: %d\n", index, frontier.cells.size());
    mid_point = frontier.cells[index];
    printf("Mid point of frontier: (%f,%f)\n", mid_point.x, mid_point.y);

    return mid_point;
}
