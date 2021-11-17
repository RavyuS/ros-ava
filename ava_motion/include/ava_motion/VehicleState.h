#include <iostream>
#include <string>
#include "ava_motion/TrackedVehicle.h"
#include "ava_motion/Vehicle.h"
#include "geometry_msgs/Point.h"
#include "VehInfo.h"
#pragma once

namespace MotionPlanning {
class State{
    public:
    virtual std::string  to_string() = 0;
    virtual State* UpdateState(const VehiclesInfo & veh_info) =0;

    protected:
    double threshold;
    double CalculateDistance(const ava_motion::VehicleConstPtr & self, const ava_motion::TrackedVehicleConstPtr & other);
    bool ValidLaneChange(std::vector<ava_motion::TrackedVehicleConstPtr> & lane_vehs, const ava_motion::TrackedVehicleConstPtr & lead_veh, double safe_gap);
};

class KeepLaneState: public State{
    std::string to_string() override {return "Keep Lane";}
    State* UpdateState(const VehiclesInfo & veh_info) override;
};

class StopState: public State{
    std::string to_string() override {return "Stop";}
    State* UpdateState(const VehiclesInfo & veh_info) override;
};

class PrepLaneChange: public State{
    std::string to_string() override {return "Prep lane change";}
    State* UpdateState(const VehiclesInfo & veh_info) override;
};

class ChangeLeft: public State{
    std::string to_string() override {return "Change left";}
    State* UpdateState(const VehiclesInfo & veh_info) override;
    int change_cnt =0;
};

class ChangeRight: public State{
    std::string to_string() override {return "Change right";}
    State* UpdateState(const VehiclesInfo & veh_info) override;  
    int change_cnt =0;  
};

class FollowLeader: public State{
    std::string to_string() override {return "Follow leader";}
    State* UpdateState(const VehiclesInfo & veh_info) override;
};

}

