//
// Created by Matthew Jones on 9/8/17.
//

#include <vector>
#include <cmath>
#include "vehicle.h"
#include "waypoints.h"

using namespace::std;

/**
 * Get the next vehicle in the current lane
 * @param vehicles - The list of all vehicles we have
 * @param lane - The current lane we're looking at
 * @return the vehicle that is directly in front of the ego car.  Or Null
 */
Vehicle* Vehicle::getNextVehicleInLane(vector <Vehicle> &vehicles, int lane) {
    Vehicle *closest = nullptr;
    double distance = std::numeric_limits<double>::max();
    for (auto &v : vehicles) {
        if (v.inLane(lane)) {
            double v_dist = getDistance(v.getS());
            if (v_dist < distance) {
                closest = &v;
                distance = v_dist;
            }
        }
    }
    return closest;
}

/**
 * Grab the distance between the current vehicle and the passed in S value
 * @param s - S to get distance to
 * @return Distance to S
 */
double Vehicle::getDistance(double s) {
    if (s < (this->s - CAR_SIZE)) {
        s += Waypoints::TRACK_SIZE;
    }
    return s - this->s;
}

/**
 * Grab the distance between two vehicles
 * @param vehicle - The vehicle to compare against
 * @return distance
 */
double Vehicle::getDistance(Vehicle vehicle) {
    return getDistance(vehicle.getS());
}

/**
 * Grab the distance between the current vehicle and the next one in the passed in lane
 * @param vehicles - List of all vehicles to compare against
 * @param lane - Lane to filter out
 * @return distance to the next vehicle
 */
double Vehicle::getDistanceToNextVehicleInLane(vector <Vehicle> &vehicles, int lane) {
    if (lane < 0 || lane >= Waypoints::MAX_LANES) {
        return -1;
    }

    Vehicle *v = getNextVehicleInLane(vehicles, lane);
    double distance = Waypoints::TRACK_SIZE;
    if (v != nullptr) {
        distance = getDistance(*v);
    }
    return distance;
}

double Vehicle::getID() const {
    return id;
}

double Vehicle::getX() const {
    return x;
}

double Vehicle::getY() const {
    return y;
}

double Vehicle::getVx() const {
    return vx;
}

double Vehicle::getVy() const {
    return vy;
}

double Vehicle::getS() const {
    return s;
}

double Vehicle::getD() const {
    return d;
}

void Vehicle::setS(double s) {
    this->s = s;
}

void Vehicle::setD(double d) {
    this->d = d;
}

int Vehicle::getLane() {
        return (int)floor(((this->d - 2.0) / 4.0 + 0.5));
}

bool Vehicle::inLane(int i) {
    return getLane() == i;
}

double Vehicle::getSpeed() {
    return sqrt( vx * vx + vy * vy );
}

