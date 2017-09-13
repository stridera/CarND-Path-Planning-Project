//
// Created by Matthew Jones on 9/6/17.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>
#include "vehicle.h"
#include "waypoints.h"

using namespace::std;

class Path_Planner {
private:
    enum states {
        STAY_IN_LANE,
        PREPARE_TO_CHANGE_LANE_LEFT,
        CHANGE_LANE_LEFT,
        PREPARE_TO_CHANGE_LANE_RIGHT,
        CHANGE_LANE_RIGHT

    };

    Waypoints waypoints;

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;

    int velocity;

    int currentState;


public:
    Path_Planner();

    void update(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                double end_path_d, vector<Vehicle> vehicles);

    std::vector<double> get_next_x();
    std::vector<double> get_next_y();

    void classroom_approach(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y,
                            double end_path_s, double end_path_d, vector<Vehicle> vehicles);

    void iterative_approach(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y,
                            double end_path_s, double end_path_d, vector<Vehicle> vehicles);
};


#endif //PATH_PLANNING_PATH_PLANNER_H
