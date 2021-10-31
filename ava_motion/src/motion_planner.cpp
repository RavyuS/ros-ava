#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ava_motion/Vehicle.h"
#include "ava_motion/TrackedVehicle.h"
#include "ava_motion/VehicleState.h"
#include "ava_motion/VehInfo.h"
#include <string>
#include <vector>

class StateManager{
    public:

    void SelfCallBack(const ava_motion::VehicleConstPtr & veh){
        veh_info.self_veh = veh;
    }

    void ObjectCallBack(const ava_motion::TrackedVehicleConstPtr & veh){

    }

    std::string GetCurrentState(){
        return cur_state->to_string();
    }

    void MotionPlanningLoop(){

    }
    
    private:
    
    VehiclesInfo veh_info;
    State* cur_state = new KeepLaneState();
    


};

int main(int argc, char **argv){
    StateManager sm;
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle n;
    ros::Subscriber self_info = n.subscribe("self_info",1, &StateManager::SelfCallBack, &sm);
    ros::Subscriber other_info = n.subscribe("detected_vehicle",1, &StateManager::ObjectCallBack, &sm);
    ros::Publisher state_pub = n.advertise<std_msgs::String>("vehicle_state", 1000);

    while(ros::ok()){
        ros::spinOnce();
        sm.MotionPlanningLoop();
        std_msgs::String msg;
        msg.data = sm.GetCurrentState().c_str();
        state_pub.publish(msg);
    }

}