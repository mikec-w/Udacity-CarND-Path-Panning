#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


struct lane_ahead
{
    /* data */
    float dist_to_car_ahead;
    float velocity_of_car_ahead;
};


// Function to tell us if the lane ahead is clear
lane_ahead IsLaneClear(int lane, double car_s, nlohmann::json sensor_fusion, int prev_size, float timestep)
{
    // default values - note this means we only look 150m ahead MAX
    float dist_to_car_ahead = 150;
    float velocity_of_car_ahead = 99;

    //Find ref_v to use
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        // find cars in my lane
        float d = sensor_fusion[i][6];
        if (d < (2+4*lane+2) && d > (2+4*lane-2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // if using prev points project s out
            check_car_s+=((double)prev_size*timestep*check_speed);

            // Check s values are greater than mine (i.e. it is in front)
            if (check_car_s > car_s && (check_car_s - car_s) < dist_to_car_ahead)
            {
                dist_to_car_ahead = check_car_s - car_s;
                velocity_of_car_ahead = check_speed;
            }
    
        }
    }

    // return
    return (lane_ahead{ dist_to_car_ahead, velocity_of_car_ahead});
}


// Get average of vector
double mean ( vector<double>& v )
{
        double return_value = 0.0;
        int n = v.size();
       
        for ( int i=0; i < n; i++)
        {
            return_value += v[i];
        }
       
        return ( return_value / n);
}

// Get max of vector
double max ( vector<double>& v )
{
        double return_value = std::numeric_limits<double>::min();
        int n = v.size();
       
        for ( int i=0; i < n; i++)
        {
            if (v[i] > return_value)
            {
                return_value = v[i];
            }
        }
       
        return ( return_value / n);
}

//Get index of max value
int MaxValInd( vector<double>& v)
{
    double max = std::numeric_limits<double>::min();
    int ind = 0;

    for (int i = 0; i < v.size(); i++)
    {
        if (v[i] > max)
        {
            max = v[i];
            ind = i;
        }
    }
    return(ind);
}

// Function to tell us average lane speed 

// Returns average speed of cars in each lane or 99 if empty
vector<double> AveLaneSpeed(int car_s, nlohmann::json sensor_fusion, int prev_size, float timestep)
{
    // default values
    vector<double> speedsLane0;
    vector<double> speedsLane1;
    vector<double> speedsLane2;

    //Find ref_v to use
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        // find cars in my lane
        float d = sensor_fusion[i][6];
        int lane = floor(d / 4);

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];

        // if using prev points project s out
        check_car_s+=((double)prev_size*timestep*check_speed);

        // Check cars that are within 50 metres of our car or 10 metres behind...
        double dist_to_car = check_car_s - car_s;
        if (dist_to_car < 50 && dist_to_car > -10)
        {
            if (lane == 0) speedsLane0.push_back(check_speed);
            if (lane == 1) speedsLane1.push_back(check_speed);
            if (lane == 2) speedsLane2.push_back(check_speed);
        }
    }

    // Check empty lanes - if all empty - favour right
    if (speedsLane0.empty()) speedsLane0.push_back(99.7);
    if (speedsLane1.empty()) speedsLane1.push_back(99.8);
    if (speedsLane2.empty()) speedsLane2.push_back(99.9);
    // return
    vector<double> retval;

    retval.push_back(mean(speedsLane0));
    retval.push_back(mean(speedsLane1));
    retval.push_back(mean(speedsLane2));

    return (retval);
}

// Function to find a safe gap to move into
bool IsSafeGap(int lane, int car_s, nlohmann::json sensor_fusion, int prev_size, float timestep)
{
    // default values
    float min_gap_behind = 10;
    float min_gap_ahead = 30;

    bool IsSafeGap = true;

    //Find ref_v to use
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        // find cars in my lane
        float d = sensor_fusion[i][6];
        if (d < (2+4*lane+2) && d > (2+4*lane-2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // if using prev points project s out
            check_car_s+=((double)prev_size*timestep*check_speed);

            // Check s values are greater than mine (i.e. it is in front)
            float dist_to_me = check_car_s - car_s;
            if (dist_to_me > -min_gap_behind && dist_to_me < min_gap_ahead)
            {
              IsSafeGap = false;
            }
        }
    }

    return(IsSafeGap);
}

