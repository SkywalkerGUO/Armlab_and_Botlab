#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    //////////// TODO: initialize the dstances for the obstacle distance grid
    // Using the map initializes distances with the cells vector
    int height = map.heightInCells();
    int width = map.widthInCells();
    for (int y=0; y < height; y++){
        for (int x=0; x < width; x++){
            if(map.logOdds(x,y) < 0){ // if free
                distance(x, y) = -1; // -1 means we haven't looked at it yet
            }
            else{
                distance(x, y) = 0; // occupied set to 0
            }
        }
    }
    
    return;
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    resetGrid(map);
    initializeDistances(map);

    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(map, *this, searchQueue);

    while (!(searchQueue.empty()))
    {
        auto nextNode = searchQueue.top();
        searchQueue.pop();
        expand_node(nextNode, *this, searchQueue);
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

void ObstacleDistanceGrid::enqueue_obstacle_cells(const OccupancyGrid& map, 
                                ObstacleDistanceGrid& grid, 
                                std::priority_queue<DistanceNode>& search_queue)
{
    ///////// TODO: Implement the method for enqueing neighboring cells
    int width = grid.widthInCells();
    int height = grid.heightInCells();
    cell_t cell;
    
    for (cell.y = 0; cell.y < height; cell.y++){
        for (cell.x = 0; cell.x < width; cell.x++){
            if(distance(cell.x, cell.y) == 0){
                expand_node(DistanceNode(cell, 0), grid, search_queue);
            }
        }
    }

    return;
}

void expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& search_queue)
{
    const int xDeltasAdj[4] = {1, 0, 0, -1}; // adjacent deltas
    const int yDeltasAdj[4] = {0, -1, 1, 0};
    const int xDeltasDiag[4] = {1, 1, -1, -1}; // diagonal deltas
    const int yDeltasDiag[4] = {1, -1, 1, -1};

    // Adjacent Cells
    for(int i = 0; i < 4; i++){
        cell_t adjacentCell(node.cell.x + xDeltasAdj[i], node.cell.y + yDeltasAdj[i]);
        if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y)){ // if in grid
            if(grid(adjacentCell.x, adjacentCell.y) == -1){ // check if we've seen it before, -1 means never seen before
                DistanceNode adjacentNode(adjacentCell, node.distance + 1); // + 1 for adjacent, 1.414 for diagonal
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                search_queue.push(adjacentNode);
            }
        }
    }

    // Diagonal Cells
    for(int i = 0; i < 4; i++){
        cell_t diagonalCell(node.cell.x + xDeltasDiag[i], node.cell.y + yDeltasDiag[i]);
        if(grid.isCellInGrid(diagonalCell.x, diagonalCell.y)){ // if in grid
            if(grid(diagonalCell.x, diagonalCell.y) == -1){ // check if we've seen it before, -1 means never seen before
                DistanceNode diagonalNode(diagonalCell, node.distance + 1.414); // 1.414 for diagonal, 1 to pass test
                grid(diagonalCell.x, diagonalCell.y) = diagonalNode.distance * grid.metersPerCell();
                search_queue.push(diagonalNode);
            }
        }
    }
}

bool is_cell_free(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) < 0;
}

bool is_cell_occupied(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) >= 0;
}
