#include "ava_motion/Vehicle.h"
#include "ava_motion/TrackedVehicle.h"
#pragma once
struct VehiclesInfo{
        ava_motion::VehicleConstPtr self_veh;
        ava_motion::TrackedVehicleConstPtr lead_veh;
        ava_motion::TrackedVehicleConstPtr lag_veh;
        std::vector<ava_motion::TrackedVehicleConstPtr> left_vehs;
        std::vector<ava_motion::TrackedVehicleConstPtr> right_vehs;
};