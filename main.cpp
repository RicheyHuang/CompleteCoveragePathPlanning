#include <iostream>
#include <map>
#include <vector>
#include <deque>
#include <algorithm>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

double INF = 10000;

int MAP_HEIGHT = 131, MAP_WIDTH = 131;

class Position2D
{
public:
    Position2D(int pos_x, int pos_y)
    {
        x = pos_x;
        y = pos_y;
    };

    int x = 0;
    int y = 0;
};

struct mapPoint
{
    double     occupancy    = INF;
    double     cost         = 0.0;
    double     exitCost     = 0.0;
    bool       isVisited    = false;
    bool       costComputed = false;
    bool       exitChecked  = false;
    bool       isTracked    = false;

    int        visitedNum   = 0;

    Position2D  next_point_position = Position2D(INF, INF);
    Position2D  prev_point_position = Position2D(INF, INF);
};


bool operator<(const Position2D &p1, const Position2D &p2) {
    return (p1.x < p2.x) || ((p1.x == p2.x) && (p1.y < p2.y));
}

bool operator == (const Position2D &p1, const Position2D &p2)
{
    return (p1.x == p2.x && p1.y == p2.y);
}


std::map<Position2D, mapPoint> prob_map;   // position-point

std::deque<Position2D> candidate_positions;

std::vector<Position2D> optimal_path;


std::vector<Position2D> getNeighbors(Position2D root_position)
{
    std::vector<Position2D> neighbor_positions = {
                Position2D(root_position.x-1  ,root_position.y-1),
                Position2D(root_position.x    ,root_position.y-1),
                Position2D(root_position.x+1  ,root_position.y-1),
                Position2D(root_position.x-1  ,root_position.y),

                Position2D(root_position.x+1  ,root_position.y),
                Position2D(root_position.x-1  ,root_position.y+1),
                Position2D(root_position.x    ,root_position.y+1),
                Position2D(root_position.x+1  ,root_position.y+1)
    };
    return neighbor_positions;
}


void updateNeighbors(Position2D root_position)
{
    std::vector<Position2D> neighbor_positions = getNeighbors(root_position);

    for(int i = 0; i < 8; i++)
    {
        if(!(0 <= neighbor_positions[i].x <= (MAP_HEIGHT-1) && 0 <= neighbor_positions[i].y <= (MAP_WIDTH-1)))
        {
            continue;
        }
        else if(prob_map[neighbor_positions[i]].occupancy != INF){

            if(!prob_map[neighbor_positions[i]].costComputed)
            {
                prob_map[neighbor_positions[i]].cost = prob_map[root_position].cost + 1;
                prob_map[neighbor_positions[i]].costComputed = true;
                candidate_positions.push_back(neighbor_positions[i]);
            }
        }
    }
}


void waveFrontSpread(Position2D start_position)
{
    prob_map[start_position].costComputed = true;
    Position2D curr_position = start_position;
    updateNeighbors(curr_position);

    while (!candidate_positions.empty())
    {
        curr_position = candidate_positions.front();
        candidate_positions.pop_front();
        updateNeighbors(curr_position);
    }
}

std::deque<Position2D> shortcut_candidate_positions;

std::deque<Position2D> sub_path;

bool isExitFound = false;
bool isPlanningFinished = false;


void resetExitCost()
{
    for(int i = 0; i < MAP_WIDTH; i++)
    {
        for (int j = 0; j < MAP_HEIGHT; j++)
        {
            prob_map[Position2D(i,j)].exitChecked = false;
            prob_map[Position2D(i,j)].exitCost = 0.0;
            prob_map[Position2D(i,j)].isTracked = false;
            shortcut_candidate_positions.clear();
            sub_path.clear();
        }
    }
}


Position2D findExit(Position2D stuck_position)
{
    std::vector<Position2D> neighbor_positions = getNeighbors(stuck_position);

    Position2D exit_point = Position2D(INF, INF);

    for(int i = 0; i < 8; i++)
    {
        if(!(0 <= neighbor_positions[i].x <= (MAP_HEIGHT-1) && 0 <= neighbor_positions[i].y <= (MAP_WIDTH-1)))
        {
            continue;
        }

        else if(prob_map[neighbor_positions[i]].occupancy != INF && !prob_map[neighbor_positions[i]].isVisited)
        {
            isExitFound  = true;
            prob_map[neighbor_positions[i]].prev_point_position = stuck_position;

            exit_point = neighbor_positions[i];
            break;
        }

        else if(prob_map[neighbor_positions[i]].occupancy != INF && !prob_map[neighbor_positions[i]].exitChecked)
        {
            prob_map[neighbor_positions[i]].exitChecked = true;
            prob_map[neighbor_positions[i]].exitCost = prob_map[stuck_position].exitCost + 1;
            prob_map[neighbor_positions[i]].prev_point_position = stuck_position;

            shortcut_candidate_positions.push_back(neighbor_positions[i]);
        }
    }

    return exit_point;
}



void cornerEscape(Position2D stuck_position)  // use dijkstra algorithm
{
    prob_map[stuck_position].exitChecked = true;
    prob_map[stuck_position].isTracked = true;

    Position2D curr_position = stuck_position;
    Position2D found_point = findExit(curr_position);   // won't find a exit in the first iteration since it is on the stuck point
//    std::cout << "shortcut_candidate_positions:" << shortcut_candidate_positions.size() << std::endl;

    while(!shortcut_candidate_positions.empty())
    {
        curr_position = shortcut_candidate_positions.front();
        shortcut_candidate_positions.pop_front();

        found_point = findExit(curr_position);

        if(found_point.x != INF && found_point.y != INF)
        {


            for(Position2D position = found_point;
                prob_map[position].prev_point_position.x != INF && prob_map[position].prev_point_position.y != INF, !prob_map[position].isTracked;
                position = prob_map[position].prev_point_position)
            {
                prob_map[position].isVisited = true;
                prob_map[position].visitedNum++;
                prob_map[position].isTracked = true;

                sub_path.push_front(position);
            }


            // for debugging
//            for(int i = 0; i < sub_path.size(); i++)
//            {
//                std::cout << sub_path[i].x << ", " << sub_path[i].y << std::endl;
//            }

            optimal_path.insert(optimal_path.end(), sub_path.begin(), sub_path.end());

            isExitFound = false; // reset the indicator
            resetExitCost();
            return;
        }

    }

    isPlanningFinished = true;
    std::cout << "path planning finished." << std::endl;
}


void findNextStep(Position2D position)
{
    bool isStuck = true;
    double min_cost = INF;

    Position2D next_position = position;

    std::vector<Position2D> neighbor_positions = getNeighbors(position);

    for(int i = 0; i < 8; i++)
    {
        if(!(0 <= neighbor_positions[i].x <= (MAP_HEIGHT-1) && 0 <= neighbor_positions[i].y <= (MAP_WIDTH-1)))
        {
            continue;
        }
        else if((prob_map[neighbor_positions[i]].occupancy != INF) && (!prob_map[neighbor_positions[i]].isVisited))
        {
            if(prob_map[neighbor_positions[i]].cost < min_cost)
            {
                min_cost = prob_map[neighbor_positions[i]].cost;
                next_position = neighbor_positions[i];

                isStuck = false;
            }
        }
    }
    if(isStuck)
    {
//        std::cout << "stuck point: " << position.x << ", " << position.y << std::endl;
        cornerEscape(position);
    }
    else
    {
        prob_map[next_position].isVisited = true;
        prob_map[next_position].visitedNum++;
        optimal_path.push_back(next_position);
//        std::cout << next_position.x << ", " << next_position.y << std::endl;
    }
}


void completeCoveragePathPlanning(Position2D start_position)
{
    waveFrontSpread(start_position);

    prob_map[start_position].isVisited = true;
    prob_map[start_position].visitedNum++;
    optimal_path.push_back(start_position);

    Position2D curr_position = start_position;

    while(!isPlanningFinished)
    {
        findNextStep(curr_position);
        curr_position = optimal_path.back();
    }
}


int main() {
//    cv::Mat map = cv::imread("ccpp.jpg", cv::IMREAD_GRAYSCALE);
//    cv::Mat binary_map = map.clone();
//    cv::threshold(map, binary_map, 175, 255, CV_THRESH_BINARY);
//    cv::imshow("thresh", binary_map);
//    cv::waitKey();

//    std::cout<<map.size[0] << " "<<map.size[1] << std::endl;


//    for(int x = 0; x < map.cols; x++)
//    {
//        for (int y = 0; y < map.rows; y++)
//        {
//            if(static_cast<double>(map.at<uchar>(y,x)) == 255)
//            {
//                prob_map[Position2D(x,y)].occupancy = 1.0;
//                std::cout << "x:" << x << ", y:" << y << ", occupancy: " << prob_map[Position2D(x,y)].occupancy << std::endl;
//            } else
//            {
//                prob_map[Position2D(x,y)].occupancy = INF;
//                std::cout << "x:" << x << ", y:" << y << ", occupancy: " << prob_map[Position2D(x,y)].occupancy << std::endl;
//            }
//            std::cout << static_cast<double>(map.at<uchar>(y,x)) << std::endl;
//        }
//    }




    // build a simple map

    for(int i = 0; i < MAP_WIDTH; i++)
    {
        for (int j = 0; j < MAP_HEIGHT; j++)
        {
            if(i != 0 && i != (MAP_WIDTH-1) && j != 0 && j != (MAP_HEIGHT-1))
            {
                prob_map[Position2D(i,j)].occupancy = 1.0;
            }
            else
            {
                prob_map[Position2D(i,j)].occupancy = INF;
                prob_map[Position2D(i,j)].cost = INF;
            }
        }
    }

    for(int i = 40; i < 90; i++)
    {
        for (int j = 40; j < 90; j++)
        {
            if(i == 40 || i == 89 || j == 89)
            {
                prob_map[Position2D(i,j)].occupancy = INF;
                prob_map[Position2D(i,j)].cost = INF;
            }
        }
    }

    Position2D start = Position2D(41,88);

    completeCoveragePathPlanning(start);

    std::cout << std::endl << std::endl;


    cv::Mat cost_map = cv::Mat::zeros(MAP_HEIGHT, MAP_WIDTH, CV_32FC1);
    for(int i = 0; i < MAP_WIDTH; i++)
    {
        for (int j = 0; j < MAP_HEIGHT; j++)
        {
            if(i != 0 && i != (MAP_HEIGHT-1) && j != 0 && j != (MAP_WIDTH-1))
            {
                cost_map.at<float>(j,i) = prob_map[Position2D(i,j)].cost*1.2;
            }
            else
            {
                cost_map.at<float>(j,i) = 255.0;
            }
        }
    }
    for(int i = 40; i < 90; i++)
    {
        for (int j = 40; j < 90; j++)
        {
            if(i == 40 || i == 89 || j == 89)
            {
                cost_map.at<float>(j,i) = 255.0;
            }
        }
    }
    cost_map.convertTo(cost_map, CV_8UC1);
    cv::applyColorMap(cost_map, cost_map, cv::COLORMAP_RAINBOW);
    cv::imshow("cost map", cost_map);
    cv::waitKey(0);


//    std::cout << "occupancy value of each position:" << std::endl << std::endl;
//
//    for(int i = 0; i < MAP_HEIGHT; i++)
//    {
//        for (int j = 0; j < MAP_WIDTH; j++)
//        {
//            if(prob_map[Position2D(j,i)].cost == INF)
//            {
//                std::cout << std::left << std::setw(2) << "*" << "  ";
//            }
////            else if(std::find(optimal_path.begin(),optimal_path.end(),Position2D(j,i)) != optimal_path.end())
////            {
////                std::cout << std::left << std::setw(2) << "$" << "  ";
////            }
//            else
//            {
//                std::cout << std::left << std::setw(2) << prob_map[Position2D(j,i)].cost << "  ";
//            }
//
//        }
//        std::cout << std::endl;
//    }
//
//    std::cout << std::endl;

//    std::cout<< "visited state of each position:" << std::endl << std::endl;
//
//    for(int i = 0; i < 13; i++)
//    {
//        for (int j = 0; j < 13; j++)
//        {
//            if(prob_map[Position2D(j,i)].cost == INF)
//            {
//                std::cout << std::left << std::setw(2) << "*" << "  ";
//            }
//            else
//            {
//                std::cout << std::left << std::setw(2) << prob_map[Position2D(j,i)].isVisited << "  ";
//            }
//
//        }
//        std::cout << std::endl;
//    }
//
//    std::cout << std::endl;

//    std::cout << "visited number of each position:" << std::endl << std::endl;
//
//    for(int i = 0; i < MAP_HEIGHT; i++)
//    {
//        for (int j = 0; j < MAP_WIDTH; j++)
//        {
//            if(prob_map[Position2D(j,i)].cost == INF)
//            {
//                std::cout << std::left << std::setw(2) << "*" << "  ";
//            }
//            else
//            {
//                std::cout << std::left << std::setw(2) << prob_map[Position2D(j,i)].visitedNum << "  ";
//            }
//
//        }
//        std::cout << std::endl;
//    }



//    for(int i = 0; i < MAP_HEIGHT; i++)
//    {
//        for (int j = 0; j < MAP_WIDTH; j++)
//        {
//            if(!prob_map[Position2D(j,i)].isVisited)
//            {
//                std::cout << "unvisited point found."<< std::endl;
//            }
//        }
//    }



    cv::Mat map = cv::Mat::zeros(MAP_HEIGHT, MAP_WIDTH, CV_32FC3);
    for(int i = 0; i < MAP_WIDTH; i++)
    {
        for (int j = 0; j < MAP_HEIGHT; j++)
        {
            if(i != 0 && i != (MAP_HEIGHT-1) && j != 0 && j != (MAP_WIDTH-1))
            {
                map.at<cv::Vec3f>(j,i) = cv::Vec3f(255.0, 255.0, 255.0);
            }
            else
            {
                map.at<cv::Vec3f>(j,i) = cv::Vec3f(0.0, 0.0, 0.0);
            }
        }
    }
    for(int i = 40; i < 90; i++)
    {
        for (int j = 40; j < 90; j++)
        {
            if(i == 40 || i == 89 || j == 89)
            {
                map.at<cv::Vec3f>(j,i) = cv::Vec3f(0.0, 0.0, 0.0);
            }
            else
            {
                map.at<cv::Vec3f>(j,i) = cv::Vec3f(255.0, 255.0, 255.0);
            }
        }
    }


    cv::Mat hotmap = cv::Mat::zeros(map.rows, map.cols, CV_32FC1);

    std::vector<Position2D>::iterator it = optimal_path.begin();

    while(it != optimal_path.end())
    {
        map.at<cv::Vec3f>(it->y, it->x) = cv::Vec3f(0.0, 0.0, 255.0);

        hotmap.at<float>(it->y, it->x) += 51.0;

        cv::imshow("trajectory", map);

        cv::waitKey(1);
        it++;
    }
    cv::waitKey(0);

    hotmap.convertTo(hotmap, CV_8UC1);

    cv::applyColorMap(hotmap, hotmap, cv::COLORMAP_HOT);
    cv::imshow("hotmap", hotmap);
    cv::waitKey(0);

    return 0;
}