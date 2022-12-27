#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

using namespace std;

/*
Dieser Code ist von meiner MOR-Abgabe 2 zur linearen Reglung adaptiert und auf den Use case dieser Übung angepasst.
Eigenplagiat mr18b070
*/

// Hier werden die Regelparameter definiert.
#define K_P 1.0
#define K_A 1.5
#define K_B -0.2

// Maximalgeschwindigkeiten des Roboters werden eingestellt (anhand der Daten der turtlebot3 Teleop Node).
#define MAX_SPEED 0.22
#define MAX_SPEED_ANG 2.84

// Eine Toleranz in welcher der nächste Punkt anvisiert wird.
#define TOLERANCE 0.1

// Hier wird die Klasse deklariert die sich um alles kümmert.
class CtrlPubSub{

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;

    geometry_msgs::Twist ctrl_cmd;
    nav_msgs::Path path;
    geometry_msgs::Pose2D goal_pose;

    bool goal_reached = false;
    bool ready = false;
    int current_goal = 0;

public:
    CtrlPubSub(){
        
        // Initialisieren des publishers für die Control Commands. sowie des Subscribers für die aktuelle Pose und des zu folgeden Pfads.
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        pose_sub = nh.subscribe("/odom", 1, &CtrlPubSub::pose_cb, this);
        path_sub = nh.subscribe("/BreadthSearchPath", 1, &CtrlPubSub::path_cb, this);
    }

    void path_cb(const nav_msgs::Path::ConstPtr& path_msg){
        
        // Hier wird der Pfad gespeichert und das Signal gegeben, dass ein Pfad vorhanden ist.
        path.header = path_msg->header;
        path.poses = path_msg->poses;

        ready = true;
        ROS_INFO_STREAM("Path received!");

    }

    void pose_cb(const nav_msgs::Odometry::ConstPtr& odom_msg){
        
        // Warten bis ein Pfad vorhanden ist.
        if(ready){
            // Da nicht mehr benötigt wird der Subscriber für den Pfad geschlossen.
            path_sub.shutdown();

            // Setzt das aktuelle Ziel je nach aktueller Position auf dem Pfad.
            goal_pose.x = path.poses[current_goal].pose.position.x;
            goal_pose.y = path.poses[current_goal].pose.position.y;
            goal_pose.theta = tf::getYaw(path.poses[current_goal].pose.orientation);

            // Der Orientierungswinkel wird auf ]-pi ; pi] beschränkt.
            if(goal_pose.theta >= M_PI){
                goal_pose.theta = -2*M_PI + goal_pose.theta; 
            }else if(goal_pose.theta < -M_PI){
                goal_pose.theta = 2*M_PI + goal_pose.theta;
            }

            // Initialisieren der aktuellen Pose des Turtlebot.
            geometry_msgs::Pose2D curr_pose;
            curr_pose.x = odom_msg->pose.pose.position.x;
            curr_pose.y = odom_msg->pose.pose.position.y;
            curr_pose.theta = tf::getYaw(odom_msg->pose.pose.orientation);

            // Der Orientierungswinkel wird auf ]-pi ; pi] beschränkt.
            if(curr_pose.theta >= M_PI){
                curr_pose.theta = -2*M_PI + curr_pose.theta; 
            }else if(curr_pose.theta < -M_PI){
                curr_pose.theta = 2*M_PI + curr_pose.theta;
            }

            // Hier werden Alpha Rho und Beta wie laut skript "lineare Regelung MOR" berechnet und die Winkel ebenso begrenzt.
            double x_diff = goal_pose.x - curr_pose.x;
            double y_diff = goal_pose.y - curr_pose.y;

            double rho = sqrt((x_diff*x_diff) + (y_diff*y_diff));
            
            double alpha = -curr_pose.theta +atan2(y_diff, x_diff);
            if(alpha >= M_PI){
                alpha = -2*M_PI + alpha; 
            }else if(alpha < -M_PI){
                alpha = 2*M_PI + alpha;
            }

            double beta = -curr_pose.theta -alpha +goal_pose.theta;
            if(beta >= M_PI){
                beta = -2*M_PI + beta; 
            }else if(beta < -M_PI){
                beta = 2*M_PI + beta;
            }

            // Hier wird die erforderliche lineare Geschwindigkeit berechnet und auf MAX_SPEED limitiert.
            double v_target = K_P * rho;
            if(v_target > MAX_SPEED){
                v_target = MAX_SPEED;
            }

            // Hier wird die erforderliche Winkelgeschwindigkeit berechnet und auf MAX_SPEED_ANG limitiert.
            double w_target = K_A * alpha + K_B * beta;
            if(w_target >= MAX_SPEED_ANG){
                w_target = MAX_SPEED_ANG;
            }else if(w_target < -MAX_SPEED_ANG){
                w_target = -MAX_SPEED_ANG;
            }

            // Der Controlmessage werden die zugehörigen Werte zugeteilt.
            ctrl_cmd.linear.x = v_target;
            ctrl_cmd.linear.y = 0;
            ctrl_cmd.linear.z = 0;
            ctrl_cmd.angular.x = 0;
            ctrl_cmd.angular.y = 0;
            ctrl_cmd.angular.z = w_target;

            // Wenn der Turtlebot innerhalb der Toleranz zum aktuellen Ziel ist, wird das nächste Ziel anvisiert.
            if(rho < TOLERANCE){  
                current_goal++;
            }

            // Wenn das Ziel erreicht ist, werden alle Geschwindigkeiten auf 0 gesetzt und die Node schließt sich.
            if(current_goal == path.poses.size()){

                ctrl_cmd.linear.x = 0;
                ctrl_cmd.linear.y = 0;
                ctrl_cmd.linear.z = 0;
                ctrl_cmd.angular.x = 0;
                ctrl_cmd.angular.y = 0;
                ctrl_cmd.angular.z = 0;

                cmd_pub.publish(ctrl_cmd);

                ROS_INFO_STREAM("Goal reached!");
                ros::shutdown();
            }

            cmd_pub.publish(ctrl_cmd);
        }
    }
};


int main(int argc, char** argv){
    
    ros::init(argc, argv, "Cmd_pub_node");

    CtrlPubSub ControlObject;

    ros::spin();

    return 666; // Copyright by Fynn Behnke - mr18b070 😈
}
