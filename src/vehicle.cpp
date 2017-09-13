//
// Created by Matthew Jones on 9/8/17.
//

#include <vector>
#include <cmath>
#include <iostream>
#include "vehicle.h"
#include "waypoints.h"

using namespace::std;

Vehicle::Vehicle(double id, double x, double y, double vx, double vy, double fs, double fd) {
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = fs;
    this->d = fd;
}

Vehicle* Vehicle::getNextVehicleInLane(vector <Vehicle> vehicles, int lane) {
    Vehicle *closest = nullptr;
    double distance = std::numeric_limits<double>::max();
    for (auto &v : vehicles) {
        cout << "Car: " << v.getID() << " S: " << v.getS() << " D: " << v.getD() << " lane: " << v.getLane() << endl;

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

double Vehicle::getDistance(double s) {
    if (s < this->s) {
        s += Waypoints::TRACK_SIZE;
    }
    if (s < this->s) {
        cout << "WTF? " << s << endl;
    }

    return s - this->s;
}

double Vehicle::getDistance(Vehicle vehicle) {
    return getDistance(vehicle.getS());
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

