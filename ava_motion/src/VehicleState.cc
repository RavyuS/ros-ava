#include "ava_motion/VehicleState.h"
#include "ava_motion/VehInfo.h"
#include "geometry_msgs/Point.h"
#include <math.h> 

namespace MotionPlanning {
State* KeepLaneState::UpdateState(const VehiclesInfo & veh_info){
    double lead_dist = CalculateDistance(veh_info.self_veh, veh_info.lead_veh);
    double dist_thresh = (pow(veh_info.self_veh->spd, 2))/ 2*(veh_info.self_veh->acc);
    if(lead_dist < dist_thresh){
        return new StopState();
    }
    else return this;
}

State* StopState::UpdateState(const VehiclesInfo & veh_info){
    return this;
}

double State::CalculateDistance(const ava_motion::VehicleConstPtr & self, const ava_motion::TrackedVehicleConstPtr & other){
    geometry_msgs::Point self_p = self->location;
    geometry_msgs::Point other_p = other->vehicle.location;

    return sqrt(pow(other_p.x - self_p.x, 2) + pow(other_p.y - self_p.y, 2));
}
}