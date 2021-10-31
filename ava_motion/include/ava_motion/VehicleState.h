#include <iostream>
#include <string>
#include "ava_motion/TrackedVehicle.h"
#include "ava_motion/Vehicle.h"
#include "geometry_msgs/Point.h"
#include "VehInfo.h"

class State{
    public:
    virtual std::string  to_string() = 0;
    virtual State* UpdateState(const VehiclesInfo & veh_info) =0;

    private:
    double threshold;
    double CalculateDistance(const ava_motion::VehicleConstPtr & self, const ava_motion::VehicleConstPtr & other);
};

class KeepLaneState: public State{
    std::string to_string() override {return "Keep Lane";}
    State* UpdateState(const VehiclesInfo & veh_info) override{return nullptr;};
};

class StopState: public State{
    std::string to_string() override {return "Stop";}
    State* UpdateState(const VehiclesInfo & veh_info) override;
};