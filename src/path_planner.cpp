//
// Created by Matthew Jones on 9/6/17.
//

#include "path_planner.h"

#include <vector>
#include <cmath>
#include <iostream>
#include "spline.h"
#include "vehicle.h"
#include "waypoints.h"

using namespace::std;

const double dist_inc = 0.5;
const double max_inc = 50;

Path_Planner::Path_Planner()
{
    velocity = 30; // We start stopped
    currentState = STAY_IN_LANE; // initial state is to stay in the lane
}

void Path_Planner::update(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                          double end_path_d, vector<Vehicle> vehicles) {
    // Always start the new path where the car is already headed
    if (previous_path_x.size() > 0) {
        me.setS(end_path_s);
        me.setD(end_path_d);
    }
    next_x_vals.clear();
    next_y_vals.clear();

    switch (currentState) {
        case STAY_IN_LANE: {
            // First, lets make sure there isn't a vehicle in front of us
            Vehicle *v = me.getNextVehicleInLane(vehicles, me.getLane());
            double distance = -1;
            double next_car_s = -1;
            double id = -1;
            double lane = -1;
            if (v != nullptr) {
                distance = me.getDistance(*v);
                next_car_s = v->getS();
                id = v->getID();
                lane = v->getLane();
            }
            cout << "(" << me.getS() << "," << next_car_s << ") id: " << id << " Distance to next car: "
                 << distance << " Lane: " << me.getLane() << ", " << lane << endl;

            // Lets drive straight ahead
            // List of widely spaced waypoints, will be using this for interpolation using spline

            if (distance > 5) {


    //            classroom_approach(me, previous_path_x, previous_path_y, end_path_s, end_path_d, vehicles);
                iterative_approach(me, previous_path_x, previous_path_y, end_path_s, end_path_d, vehicles);
            }

//            cout << "X: ";
//            for (int j = 0; j < next_x_vals.size(); ++j) {
//                cout << next_x_vals[j] << " ";
//            }
//            cout << endl << "Y: ";
//            for (int j = 0; j < next_y_vals.size(); ++j) {
//                cout << next_y_vals[j] << " ";
//            }
//            cout << endl;
            break;
        }
        case PREPARE_TO_CHANGE_LANE_LEFT:
            break;
        case CHANGE_LANE_LEFT:
            break;
        case PREPARE_TO_CHANGE_LANE_RIGHT:
            break;
        case CHANGE_LANE_RIGHT:
            break;
    }




}

vector<double> Path_Planner::get_next_x() {
    return next_x_vals;
}

std::vector<double> Path_Planner::get_next_y() {
    return next_y_vals;
}

void Path_Planner::classroom_approach(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y,
                                      double end_path_s, double end_path_d, vector<Vehicle> vehicles) {
    vector<double> ptsx;
    vector<double> ptsy;

    int prev_size = previous_path_x.size();
    double ref_x = me.getX();
    double ref_y = me.getY();
    double ref_yaw = Waypoints::deg2rad(me.getVx());
    double car_s = me.getS();
    double lane = me.getLane();

    if(prev_size<2){

        double pre_car_x = me.getX() - cos(me.getVx());
        double pre_car_y = me.getY() - sin(me.getVx());

        ptsx.push_back(pre_car_x);
        ptsx.push_back(me.getX());

        ptsy.push_back(pre_car_y);
        ptsy.push_back(me.getY());
    } else {
        return;
    }

    vector<double> next_wp0 = waypoints.getXY(car_s+30,(2+ 4*lane));
    vector<double> next_wp1 = waypoints.getXY(car_s+60,(2+ 4*lane));
    vector<double> next_wp2 = waypoints.getXY(car_s+90,(2+ 4*lane));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);


    for(int i=0;i<ptsx.size();i++){
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = (shift_x *cos(0-ref_yaw)- shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x *sin(0-ref_yaw)+ shift_y*cos(0-ref_yaw));

    }

    // creating a spline to map the path on
    tk::spline s;

    // set x,y points to spline for a polynomial fit
    s.set_points(ptsx,ptsy);

    for(int i =0;i<previous_path_x.size();i++){
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 50.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

    double x_add_on = 0;

    // Fill up the remaining points for our path planner (considering 50 points now )
    for(int i = 1; i <= 50-previous_path_x.size(); i++){

        double N = (target_dist/(0.02*velocity/2.24));
        double x_point = x_add_on +	(target_x) / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal
        x_point = (x_ref*cos(me.getVx())-y_ref*sin(me.getVx()));
        y_point = (x_ref*sin(me.getVx())+y_ref*cos(me.getVx()));

        x_point += me.getX();
        y_point += me.getY();

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

void Path_Planner::iterative_approach(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                                      double end_path_d, vector<Vehicle> vehicles) {
    double car_s = me.getS();
    double car_d = me.getD();

    next_x_vals.clear();
    next_y_vals.clear();
    for (int i = 0; i < 50; i++) {
        vector<double> wp = waypoints.getXY(car_s+(0.1 * i),car_d);
        next_x_vals.push_back(wp[0]);
        next_y_vals.push_back(wp[1]);
    }
}

