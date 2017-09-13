//
// Created by Matthew Jones on 9/8/17.
//

#ifndef PATH_PLANNING_VEHICLE_SENSOR_DATA_H
#define PATH_PLANNING_VEHICLE_SENSOR_DATA_H


class Vehicle {
private:
    double id; // car's unique ID,
    double x;  // car's x position in map coordinates,
    double y;  // car's y position in map coordinates,
    double vx; // car's x velocity in m/s,
    double vy; // car's y velocity in m/s,
    double s;  // car's s position in frenet coordinates,
    double d;  // car's d position in frenet coordinates.

public:
    Vehicle(double id, double x, double y, double vx, double vy, double fs, double fd);

    // Setters
    void setS(double s);
    void setD(double d);

    // Functions
    double getDistance(double d);
    int getLane();
    bool inLane(int i);
    Vehicle * getNextVehicleInLane(std::vector <Vehicle> vehicles, int lane);

    double getID() const;

    double getX() const;

    double getY() const;

    double getVx() const;

    double getVy() const;

    double getS() const;

    double getD() const;

    double getDistance(Vehicle vehicle);

    double getSpeed();
};


#endif //PATH_PLANNING_VEHICLE_SENSOR_DATA_H