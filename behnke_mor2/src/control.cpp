#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

using namespace std;

#define K_P 0.3
#define K_A 0.8
#define K_B -0.4

// Maximum speeds of turtlebot
#define MAX_SPEED 0.22
#define MAX_SPEED_ANG 2.84

// Tolerance for Position
#define TOLERANCE 0.05

/*
Posen zum Stange umfahren:
P1: x=4, y=-1, th=0
P2: x=5, y=-2, th=-pi/2
P3: x=4, y=-3, th=pi
P4: x=3, y=-2, th=pi/2
P5: x=0, y=0, th=0     Zurück zum start
*/

//class initialisation to publish and subscribe in one function
class CtrlPubSub{

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber pose_sub;

public:
    CtrlPubSub(){
        // set up publisher and subscriber
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        pose_sub = nh.subscribe("/odom", 1, &CtrlPubSub::pose_cb, this);
    }

    void pose_cb(const nav_msgs::Odometry::ConstPtr& odom_msg){
                
        geometry_msgs::Pose2D curr_pose;
        curr_pose.x = odom_msg->pose.pose.position.x;
        curr_pose.y = odom_msg->pose.pose.position.y;
        curr_pose.theta = tf::getYaw(odom_msg->pose.pose.orientation); // calculate theta from orientation quaternion

        // limiting current theta to [pi ; -pi]
        if(curr_pose.theta > M_PI){
            curr_pose.theta = -2*M_PI + curr_pose.theta; 
        }else if(curr_pose.theta < -M_PI){
            curr_pose.theta = 2*M_PI + curr_pose.theta;
        }

        // get goal pose via ros parameter
        geometry_msgs::Pose2D goal_pose;
        goal_pose.x = ros::param::param<double>("~goal_x", 0);
        goal_pose.y = ros::param::param<double>("~goal_y", 0);
        goal_pose.theta = ros::param::param<double>("~goal_theta", 0);
        
        // limiting theta to [pi ; -pi]
        if(goal_pose.theta > M_PI){
            goal_pose.theta = -2*M_PI + goal_pose.theta; 
        }else if(goal_pose.theta < -M_PI){
            goal_pose.theta = 2*M_PI + goal_pose.theta;
        }

        geometry_msgs::Twist ctrl_cmd;

        double x_diff = goal_pose.x - curr_pose.x;
        double y_diff = goal_pose.y - curr_pose.y;

        double rho = sqrt((x_diff*x_diff) + (y_diff*y_diff));
        
        // calculation of alpha and limiting to [pi ; -pi]
        double alpha = -curr_pose.theta +atan2(y_diff, x_diff);
        if(alpha > M_PI){
            alpha = -2*M_PI + alpha; 
        }else if(alpha < -M_PI){
            alpha = 2*M_PI + alpha;
        }

        // calculation of beta and limiting to [pi ; -pi]
        double beta = -curr_pose.theta -alpha +goal_pose.theta;
        if(beta > M_PI){
            beta = -2*M_PI + beta; 
        }else if(beta < -M_PI){
            beta = 2*M_PI + beta;
        }

        // check if target linear speed is faster than turtlebot max linear speed
        double v_target = K_P * rho;
        if(v_target > MAX_SPEED){
            v_target = MAX_SPEED;
        }

        // check if target angular speed is faster than turtlebot max angular speed
        double w_target = K_A * alpha + K_B * beta;
        if(w_target > MAX_SPEED_ANG){
            w_target = MAX_SPEED_ANG;
        }else if(w_target < -MAX_SPEED_ANG){
            w_target = -MAX_SPEED_ANG;
        }
	
	    // built in tolerance to stop when close enough to goal position
        if(rho > TOLERANCE){
            ctrl_cmd.linear.x = v_target;
            ctrl_cmd.linear.y = 0;
            ctrl_cmd.linear.z = 0;
            ctrl_cmd.angular.x = 0;
            ctrl_cmd.angular.y = 0;
            ctrl_cmd.angular.z = w_target;     
        }else{
            ctrl_cmd.linear.x = 0;
            ctrl_cmd.linear.y = 0;
            ctrl_cmd.linear.z = 0;
            ctrl_cmd.angular.x = 0;
            ctrl_cmd.angular.y = 0;
            ctrl_cmd.angular.z = 0;

            cmd_pub.publish(ctrl_cmd);
            ros::shutdown();
        }

        cmd_pub.publish(ctrl_cmd);
    }
};


int main(int argc, char** argv){
    
    ros::init(argc, argv, "Cmd_pub_node");

    CtrlPubSub ControlObject;

    ros::spin();

    return 666; // Copyright by Fynn Behnke - mr18b070 😈
}
