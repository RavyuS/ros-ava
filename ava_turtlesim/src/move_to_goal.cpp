// #include <boost/bind.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cstdlib>
#include <cmath>


class TurtleController{
    public:
    TurtleController(double X, double Y): gx_(X), gy_(Y) {};
    
    void PoseCallBack(const turtlesim::PoseConstPtr & pose){
        
        g_pose = pose;
        
    }

    void DriveLoop(ros::Publisher & pub){
        geometry_msgs::Twist twist;

        if(!g_pose) return;

        if(IsMoving()) {
            // ROS_INFO("Is Moving");
            return; // Don't do anything while moving
        }

        else if(HasReachedGoal()){
            ROS_INFO("Reached goal!");
            ros::shutdown();
        }
        
        else if(fabs(GetTargetAngle() - g_pose->theta) > 0.05){
            float z_move = GetTargetAngle() - g_pose->theta;
            // ROS_INFO("Original angle: %f\nTarget Angle: %f", g_pose->theta,GetTargetAngle());
            if(fabs(z_move/M_PI) > 1){
                if(z_move < 0 ) z_move += 2*M_PI;
                else z_move -= 2*M_PI;
            }
            ROS_INFO("Turning: %f",z_move);
            twist.angular.z = z_move;
            pub.publish(twist);
        }
        else{
            // twist.linear.x = GetDistance() > 1? 1 : GetDistance();
            twist.linear.x = GetDistance();
            pub.publish(twist);
        }

    }

    private:

    double gx_;
    double gy_;
    turtlesim::PoseConstPtr g_pose;

    bool HasReachedGoal(){
        return fabs(g_pose->x - gx_) < 0.2 && fabs(g_pose->y - gy_) < 0.2;
    }

    bool IsMoving(){
        return fabs(g_pose->linear_velocity) > 0 || fabs(g_pose->angular_velocity) > 0;
    }

    float GetTargetAngle(){
        float d_x = gx_ - g_pose->x;
        float d_y = gy_ - g_pose->y;

        float tan_inv = atan2(d_y, d_x);
        ROS_INFO("Angle: %f, Current heading: %f", tan_inv, g_pose->theta);
        return tan_inv;
        // if(d_x > 0 && d_y > 0) return tan_inv;
        // if(d_x < 0 && d_y > 0) return M_PI + tan_inv;
        // if(d_x > 0 && d_y < 0) return tan_inv;
        // if(d_x < 0 && d_y < 0) return tan_inv - M_PI;
    }

    float GetDistance(){
        return std::sqrt(std::pow((gx_-g_pose->x),2) + std::pow((gy_-g_pose->y),2));
    }



};

int main(int argc, char **argv){
    if(argc != 3){
        ROS_INFO("usage: move_to_goal X Y");
        return 1;
    }

    TurtleController tc(atof(argv[1]),atof(argv[2]));
    ros::init(argc, argv, "move_to_goal");
    ros::NodeHandle n;
    ros::Subscriber pose = n.subscribe("turtle1/pose", 1, &TurtleController::PoseCallBack, &tc);
    ros::Publisher twist = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    
    // ros::Timer timer1 = n.createTimer(ros::Duration(1.0), boost::bind(&TurtleController::DriveLoop,_1, twist), &tc);
    // ros::spin();
    ros::Rate loop_rate(1.0);
    while(ros::ok()){
        // ROS_INFO("Reached goal!");
        ros::spinOnce();

        tc.DriveLoop(twist);
        loop_rate.sleep();
    }
}