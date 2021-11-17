#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ava_motion/Vehicle.h"
#include "ava_motion/TrackedVehicle.h"
#include "ava_motion/VehicleState.h"
#include "ava_motion/VehInfo.h"
#include <string>
#include <vector>
namespace MotionPlanning {
class StateManager{
    public:

    void SelfCallBack(const ava_motion::VehicleConstPtr & veh){
        veh_info_.self_veh = veh;
    }

    void ObjectCallBack(const ava_motion::TrackedVehicleConstPtr & veh){
            /**
            Logic that determines what category tracked vehicle falls in to (Lead vehicle, lag vehicle, or left/right vehicle)
            **/

            // Lead vehicle
            if(veh->vehicle.location.x > 0 && veh->vehicle.location.y == 0){
                if (veh_info_.lead_veh){
                    if(veh->vehicle.location.x >= veh_info_.lead_veh->vehicle.location.x) veh_info_.lead_veh = veh;
                    else if(veh->id == veh_info_.lead_veh->id){
                        veh_info_.lead_veh = veh;
                    }
                }
                else veh_info_.lead_veh = veh;
            }

            //Lag vehicle
            else if (veh->vehicle.location.x < 0 && veh->vehicle.location.y == 0){
                if (veh_info_.lag_veh){
                    if(veh->vehicle.location.x <= veh_info_.lag_veh->vehicle.location.x) veh_info_.lag_veh = veh;
                    else if(veh->id == veh_info_.lag_veh->id){
                        veh_info_.lag_veh = veh;
                    }
                }
                else veh_info_.lag_veh = veh;
            }

            //Left vehicle
            else if (veh->vehicle.location.y < 0){
                bool set = false;
                for (auto left_veh : veh_info_.left_vehs){
                    if (left_veh->id == veh->id){
                        set = true;
                        left_veh = veh;
                        break;
                    }
                }
                if(!set){
                    veh_info_.left_vehs.push_back(veh);
                }
            }
            //Right vehicle
            else if (veh->vehicle.location.y > 0){
                bool set = false;
                for (auto right_veh : veh_info_.right_vehs){
                    if (right_veh->id == veh->id){
                        set = true;
                        right_veh = veh;
                        break;
                    }
                }
                if(!set){
                    veh_info_.right_vehs.push_back(veh);
                }
            }
    }

    std::string GetCurrentState(){
        return cur_state_->to_string();
    }

    void MotionPlanningLoop(){
        State* new_state = cur_state_->UpdateState(veh_info_);
        if(new_state != cur_state_){ // state change requires memory cleanup
            delete cur_state_;
        }
        cur_state_ = new_state;

    }
    
    private:
    
    VehiclesInfo veh_info_;
    State* cur_state_ = new KeepLaneState();

   
    
    


};
}



int main(int argc, char **argv){
    MotionPlanning::StateManager sm;
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle n;
    ros::Subscriber self_info = n.subscribe("self_info",1, &MotionPlanning::StateManager::SelfCallBack, &sm);
    ros::Subscriber other_info = n.subscribe("detected_vehicle",1, &MotionPlanning::StateManager::ObjectCallBack, &sm);
    ros::Publisher state_pub = n.advertise<std_msgs::String>("vehicle_state", 1000);

    while(ros::ok()){
        ros::spinOnce();
        sm.MotionPlanningLoop();
        std_msgs::String msg;
        msg.data = sm.GetCurrentState().c_str();
        state_pub.publish(msg);
    }
}