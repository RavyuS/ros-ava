#include "ava_motion/VehicleState.h"
#include "ava_motion/VehInfo.h"
#include "geometry_msgs/Point.h"
#include <math.h> 

namespace MotionPlanning {
State* KeepLaneState::UpdateState(const VehiclesInfo & veh_info){

    if(veh_info.lead_veh){
        double lead_dist = CalculateDistance(veh_info.self_veh, veh_info.lead_veh);
        double dist_thresh = (pow(veh_info.self_veh->spd, 2))/ 2*(veh_info.self_veh->acc);

        if(lead_dist < dist_thresh){
            return new StopState();
        }
        if(lead_dist < veh_info.LEAD_DIST_THRESH){
            return new PrepLaneChange();
        }
        
    }
    
    return this;
}

State* StopState::UpdateState(const VehiclesInfo & veh_info){

    if(veh_info.lead_veh){
        double lead_dist = CalculateDistance(veh_info.self_veh, veh_info.lead_veh);

        if(lead_dist >= veh_info.LEAD_DIST_THRESH){
            return new PrepLaneChange();
        }
        return this;
    }

    else{
        return new KeepLaneState();
    }
}

State* PrepLaneChange::UpdateState(const VehiclesInfo & veh_info){
    if(veh_info.lead_veh){
        double lead_dist = CalculateDistance(veh_info.self_veh, veh_info.lead_veh);
        double dist_thresh = (pow(veh_info.self_veh->spd, 2))/ 2*(veh_info.self_veh->acc);

        if(lead_dist < dist_thresh){
            return new StopState();
        }

        if(lead_dist >= veh_info.LEAD_DIST_THRESH){
            return new FollowLeader();
        }

        //Check if lane change can be made (prioritize left)
        if(ValidLaneChange(veh_info.left_vehs, veh_info.lead_veh, veh_info.LEAD_DIST_THRESH)) return new ChangeLeft();
        else if (ValidLaneChange(veh_info.right_vehs, veh_info.lead_veh, veh_info.LEAD_DIST_THRESH)) return new ChangeRight();

        else return this;
    }
    else{
        return new KeepLaneState();
    }

}

State* ChangeLeft::Update(const VehiclesInfo & veh_info){
    if (change_cnt <5){
        ++change_cnt;
        return this;
    }
    else return new KeepLaneState();
}

State* ChangeRight::Update(const VehiclesInfo & veh_info){
    if (change_cnt <5){
        ++change_cnt;
        return this;
    }
    else return new KeepLaneState();
}

State* FollowLeader::Update(const VehiclesInfo & veh_info){
    if(veh_info.lead_veh){
        double lead_dist = CalculateDistance(veh_info.self_veh, veh_info.lead_veh);
        double dist_thresh = (pow(veh_info.self_veh->spd, 2))/ 2*(veh_info.self_veh->acc);

        if(lead_dist < dist_thresh){
            return new StopState();
        }
        if(lead_dist < veh_info.LEAD_DIST_THRESH){
            return new PrepLaneChange();
        }
        else return this;
    }
    else return new KeepLaneState();

}


double State::CalculateDistance(const ava_motion::VehicleConstPtr & self, const ava_motion::TrackedVehicleConstPtr & other){
    geometry_msgs::Point self_p = self->location;
    geometry_msgs::Point other_p = other->vehicle.location;

    return sqrt(pow(other_p.x - self_p.x, 2) + pow(other_p.y - self_p.y, 2));
}

bool State::ValidLaneChange(std::vector<ava_motion::TrackedVehicleConstPtr> & lane_vehs,const ava_motion::TrackedVehicleConstPtr & lead_veh, double safe_gap){
    
    ava_motion::TrackedVehicleConstPtr lane_lead_veh;
    double speed_factor = 1.15; //factor by which lane lead must be higher than current lead for valid lane change

    //iterate through all vehicles on lane and find lead. Fail if any vehicle is too close.

    for (auto veh : lane_vehs){
        if (abs(veh->vehicle.location.x) > safe_gap) return False;
        else{
            if (veh->vehicle.location.x > 0){
                if(!lane_lead_veh) lane_lead_veh = veh;
                else{
                    if (lane_lead_veh->vehicle.location.x > veh->vehicle.location.x) lane_lead_veh = veh;
                }
            }
        }
    }
    if(lane_lead_veh){
        if(lane_lead_veh->vehicle.spd > lead_veh->vehicle.spd * speed_factor) return true;
        else return false;
    }
    else return true;
}
}