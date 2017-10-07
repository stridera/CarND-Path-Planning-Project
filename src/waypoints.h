//
// Created by Matthew Jones on 9/7/17.
//

#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H

#include <iostream>
#include <vector>
#include <cmath>

class Waypoints {
private:
    // waypoints file to read from
    const std::string map_file_ = "../data/highway_map.csv";

    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

public:
    Waypoints();

    int ClosestWaypoint(double x, double y);
    int NextWaypoint(double x, double y, double theta);
    std::vector<double> getFrenet(double x, double y, double theta);
    std::vector<double> getXY(double s, double d);

public: // Constants
    // The max s value before wrapping around the track back to 0
    constexpr static const double TRACK_SIZE = 6945.554;
    constexpr static const int MAX_LANES = 3;

    /*
     * Math Helpers
     */
    static double pi() { return 3.14159265358979323846264338327950288; }

    static double deg2rad(double x) { return x * pi() / 180; }

    static double rad2deg(double x) { return x * 180 / pi(); }

    static double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

};
#endif //PATH_PLANNING_WAYPOINTS_H
