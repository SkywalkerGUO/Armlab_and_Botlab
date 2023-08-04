#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!initialized_){
        previousPose_ = pose;
    }
    MovingLaserScan movingScan(scan, previousPose_, pose);
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }
    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
    if (ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);
        //Now iterate through the cells and update the odds
        if (map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    if (ray.range < kMaxLaserDistance_){
        std::vector<Point<int>> cells_touched = bresenham(ray, map);

        //Lower liklihood
        for(auto& cell : cells_touched){
            if (map.isCellInGrid(cell.x, cell.y)){
                decreaseCellOdds(cell.x, cell.y, map);
            }
        }
    }
}


void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
   if(std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_) {
        map(x,y) += kHitOdds_;
    }
    else{
        map(x,y) = std::numeric_limits<CellOdds>::max();
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){
    if(map(x,y) - std::numeric_limits<CellOdds>::min() > kMissOdds_){
        map(x,y) -= kMissOdds_;
    }
    else{
        map(x,y) = std::numeric_limits<CellOdds>::min();
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + start_cell.x);
    end_cell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + start_cell.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    int dx, dy, sx, sy, err, x, y, e2;
    e2 = 0;
    int x1 = end_cell.x;
    int x0 = start_cell.x;
    int y0 = start_cell.y;
    int y1 = end_cell.y;
    dx = abs(x1-x0);
    dy = abs(y1-y0);
    sx = x0<x1 ? 1 : -1;
    sy = y0<y1 ? 1 : -1;
    err = dx-dy;
    x = x0;
    y = y0;
    while(x != x1 || y != y1){

        Point<int> new_cell;
        new_cell.x = x;
        new_cell.y = y;
        
        cells_touched.push_back(new_cell);

        e2 = 2*err;
        if (e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err += dx;
            y += sy;
        }
    }
    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}
