#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map){
    double score = 0.00001;

    double cell_length = map.metersPerCell();
    double cell_diag = sqrt(2 * cell_length * cell_length);

    // Finds the endpoint of the ray
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );
    Point<float> f_end_before = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + (ray.range - cell_diag) * std::cos(ray.theta),
            ray.origin.y + (ray.range - cell_diag) * std::sin(ray.theta)
            ), 
        map
        );
    Point<float> f_end_after = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + (ray.range + cell_diag) * std::cos(ray.theta),
            ray.origin.y + (ray.range + cell_diag) * std::sin(ray.theta)
            ), 
        map
        );

    // Stores the endpoint into a Point
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);

    Point<int> end_cell_after;
    end_cell_after.x = static_cast<int>(f_end_after.x);
    end_cell_after.y = static_cast<int>(f_end_after.y);

    Point<int> end_cell_before;
    end_cell_before.x = static_cast<int>(f_end_before.x);
    end_cell_before.y = static_cast<int>(f_end_before.y);

    if(map.logOdds(end_cell.x,end_cell.y) > 0){
        score = static_cast<double>(map.logOdds(end_cell.x,end_cell.y));
    }
    // If not positive, add 40% of logodds into the scanScore
    else{
        if(map.logOdds(end_cell_after.x,end_cell_after.y) > 0){
            score = map.logOdds(end_cell_after.x,end_cell_after.y) * 0.3;
        }
        if(map.logOdds(end_cell_before.x,end_cell_before.y) > 0||map.logOdds(end_cell_before.x,end_cell_before.y) > map.logOdds(end_cell_after.x,end_cell_after.y)){
            score = map.logOdds(end_cell_before.x,end_cell_before.y) * 0.3;
        }
    }

    return score;
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;

    for(auto& ray : movingScan){
        scanScore += scoreRay(ray, map);
    }
    return scanScore;
}