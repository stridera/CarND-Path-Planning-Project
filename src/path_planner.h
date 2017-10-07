//
// Created by Matthew Jones on 9/6/17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>
#include "vehicle.h"
#include "waypoints.h"
#include "spline.h"

using namespace::std;

class Path_Planner {
private: // CONSTANTS
    constexpr static const double MAX_VELOCITY = 4.8;
    constexpr static const double MAX_DISTANCE = 50.0;

private:
    Waypoints waypoints;

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    double velocity;

    template<typename T>
    T clamp(T n, T lower, T upper) {
        return std::max(lower, std::min(n, upper));
    }
public:
    Path_Planner(): velocity(0) {};

    void update(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                double end_path_d, vector<Vehicle> vehicles);

    std::vector<double> get_next_x();
    std::vector<double> get_next_y();

    void generate_path(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                       double end_path_d, int lane);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
