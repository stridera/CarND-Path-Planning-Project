//
// Created by Matthew Jones on 9/6/17.
//

#include "path_planner.h"

using namespace ::std;

void Path_Planner::update(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s,
                          double end_path_d, vector<Vehicle> vehicles) {
    // Start with a clean slate.
    next_x_vals.clear();
    next_y_vals.clear();

    // First, lets make sure there isn't a vehicle in front of us
    int currentLane = me.getLane();
    int targetLane = currentLane;

    double distance = me.getDistanceToNextVehicleInLane(vehicles, currentLane);

    if (distance > MAX_DISTANCE) {
        // Nothing ahead of us.  Fly like the wind!
        velocity += 0.03;
    } else {
        // If we here, we're approaching another car.  My tactic here is to check the lanes nearby and if there
        // is more space in that lane, we'll cross over.  Otherwise, we'll begin to slow down depending on how
        // close we are to the car.
        // TODO: Consider all lanes.  It would be nice at some points to do multiple lane changes to get open road.
        double rightDistance = me.getDistanceToNextVehicleInLane(vehicles, currentLane + 1);
        double leftDistance = me.getDistanceToNextVehicleInLane(vehicles, currentLane - 1);

        if (rightDistance > distance && rightDistance > leftDistance) {
            targetLane = currentLane + 1;
        } else if (leftDistance > distance && leftDistance > rightDistance) {
            targetLane = currentLane - 1;
        } else {
            // Map the velocity to the distance.
            velocity = (MAX_VELOCITY / MAX_DISTANCE) * distance;
        }
    }
    // This is just to prevent the car from going too fast or too slow.  (Causes problems with the spline)
    velocity = clamp(velocity, 1.0, MAX_VELOCITY);

    // Now we have a good idea which lane we want to be in.  Lets go there!
    generate_path(me, previous_path_x, previous_path_y, end_path_s, end_path_d, targetLane);
}

// Functions to access the path.
vector<double> Path_Planner::get_next_x() {
    return next_x_vals;
}

std::vector<double> Path_Planner::get_next_y() {
    return next_y_vals;
}

/**
 * Generate the path from where the car currently is (including the already calculated path from prior runs)
 * to where we want to be.  This takes the current location, finds where it should be using the lane paramater,
 * and then maps out a path for the car to follow.  This follows much of the explanation given in the project
 * video and the class.
 */
void Path_Planner::generate_path(Vehicle me, vector<double> previous_path_x, vector<double> previous_path_y,
                                 double end_path_s, double end_path_d, int lane) {
    // First, lets build a very rough path that we want to follow
    vector<double> ptsx;
    vector<double> ptsy;

    int prev_size = previous_path_x.size();

    double car_s = me.getS();
    double car_d = me.getD();
    double car_yaw = me.getVx();

    double ref_x = me.getX();
    double ref_y = me.getY();
    double ref_yaw = waypoints.deg2rad(car_yaw);

    if (prev_size < 2) {
        // If we're here, most likely we're on our first run.  Lets add the cars current position and a little behind us
        // so we can have a smooth start.
        double pre_car_x = me.getX() - cos(car_yaw);
        double pre_car_y = me.getY() - sin(car_yaw);

        ptsx.push_back(pre_car_x);
        ptsx.push_back(me.getX());

        ptsy.push_back(pre_car_y);
        ptsy.push_back(me.getY());
    } else {
        // Here we have a path that the car is already running from prior iterations.  We want to keep that path and only
        // add onto it.  This prevents the car from jerking around or 'teleporting' to new locations.
        car_s = end_path_s;
        car_d = end_path_d;

        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Now that we have a starting point, lets plan where we want the car to be.  We choose 30 point increments to ensure
    // there is enough room for lane changes to happen w/o the jump being too quick (and jerky) while still following the
    // road.
    vector<double> next_wp0 = waypoints.getXY(car_s + 30, 2 + 4 * lane);
    vector<double> next_wp1 = waypoints.getXY(car_s + 60, 2 + 4 * lane);
    vector<double> next_wp2 = waypoints.getXY(car_s + 90, 2 + 4 * lane);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Now we shift car coordinates to 0 degrees so we don't break the spline  (otherwise everything will be 0 degrees
    // if we're driving straight ahead and we can't pull out seperate points from the spline
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Now we take the path we have, that's full of hard angles and jerks, and smooth it out using a spline
    tk::spline s;

    // set x,y points to spline for a polynomial fit
    s.set_points(ptsx, ptsy);

    // Now we select a point ahead of us on the smooth path we've created and prepare to map everything between where
    // the previous path left off and the new path starts.  Arbitrarily select 100m as my farthest point out.
    double target_x = 100.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    // Now we have our smooth path calculated, we're ready to generate the path.  First, lets add the previous points to
    // the final path and then we'll continue from there.
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Fill up the remaining points for our path (arbitrarily considering 50 points)
    double x_add_on = 0;
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        // For each step, get the distance ahead of the last point.  The faster we're going (velocity) the more
        // space between points.
        double N = (target_dist / (0.2 * velocity / 2.24));
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Bring this back to the initial rotation that the simulator expects
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        // Finally, we add the points to the route.
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}
