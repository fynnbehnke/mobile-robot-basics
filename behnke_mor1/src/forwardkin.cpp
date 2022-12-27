#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
using namespace std;

double TIMESTEP = 0.1; // according to 10 Hz

int main(int argc, char** argv){

    // initialization of node and publisher
    ros::init(argc, argv, "Odom_pub_node");
    ros::NodeHandle nh("~");
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);

    ros::Rate loop_rate(10);

    // get Modi via Launchfile (1 = forward kinematic; 2 = ICC)
    int mode;
    nh.getParam("modi", mode);

    nav_msgs::Odometry odometry;

    odometry.header.frame_id = "map";

    // initial Position
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.pose.pose.orientation.w = 1;

    odometry.twist.twist.linear.x = 0;
    odometry.twist.twist.linear.y = 0;
    odometry.twist.twist.linear.z = 0;
    odometry.twist.twist.angular.x = 0;
    odometry.twist.twist.angular.y = 0;
    odometry.twist.twist.angular.z = 0;


    while(ros::ok()){
        // Für die Herleitung der Einzelnen Formeln siehe /documentation/Übung1_Herleitung_Behnke.pdf

        if(mode == 1){   // Vorwärtskinematik, ausgelegt auf 1.5 m/s
            
            double percent;
            nh.getParam("wheel_diff_percentage", percent);  // get wheelspeed difference over launchfile (single digit percentage)

            double base = 0.25;                         // Wheelbase
            double v = 1.5;                             // target speed

            double k = 1 - (percent/100);               // wheel ratio
            double phi_dot_r = 5;                       // predefined speed of right wheel
            double phi_dot_l = phi_dot_r * k;           // according speed of left wheel

            double rad_l = v / (phi_dot_r * k);     // necessary wheel radius left wheel
            double rad_r = rad_l * k;                   // necessary wheel radius right wheel


            //Odometriewerte zuordnen
            odometry.twist.twist.linear.x = cos(odometry.pose.pose.orientation.z)*((rad_l*phi_dot_l+rad_r*phi_dot_r)/2);
            odometry.twist.twist.linear.y = sin(odometry.pose.pose.orientation.z)*((rad_l*phi_dot_l+rad_r*phi_dot_r)/2);
            odometry.twist.twist.linear.z = 0;
            odometry.twist.twist.angular.x = 0;
            odometry.twist.twist.angular.y = 0;
            odometry.twist.twist.angular.z = ((rad_r*phi_dot_r-rad_l*phi_dot_l)/base);

            odometry.pose.pose.position.x = odometry.pose.pose.position.x + (odometry.twist.twist.linear.x * TIMESTEP);
            odometry.pose.pose.position.y = odometry.pose.pose.position.y + (odometry.twist.twist.linear.y * TIMESTEP);
            odometry.pose.pose.position.z = 0;
            odometry.pose.pose.orientation.x = 0;
            odometry.pose.pose.orientation.y = 0;
            odometry.pose.pose.orientation.z = odometry.pose.pose.orientation.z + (odometry.twist.twist.angular.z * TIMESTEP);
            odometry.pose.pose.orientation.w = 1;

            ROS_INFO_STREAM("\nWheelspeed difference [%]: " << percent << "\nWheelspeed ratio: " << k << "\nWheelradius right [m]: " << rad_r << "\nWheelradius left [m]: " << rad_l << "\n\n" << odometry.pose.pose << odometry.twist.twist);

        }else if(mode == 2){ // Mit ICR/ICC, ausgelegt auf 1.5 m/s
            double v_c = 1.5;
            double w_c = 0.0000000000000000000000000000000001;  // close to 0 for a (nearly) straight linear movement
            
            odometry.twist.twist.linear.x = ((-abs(v_c/w_c)*sin(odometry.pose.pose.orientation.z))+(abs(v_c/w_c)*sin(odometry.pose.pose.orientation.z+odometry.twist.twist.angular.z*TIMESTEP)))/TIMESTEP;
            odometry.twist.twist.linear.y = ((abs(v_c/w_c)*cos(odometry.pose.pose.orientation.z))-(abs(v_c/w_c)*cos(odometry.pose.pose.orientation.z+odometry.twist.twist.angular.z*TIMESTEP)))/TIMESTEP;
            odometry.twist.twist.linear.z = 0;
            odometry.twist.twist.angular.x = 0;
            odometry.twist.twist.angular.y = 0;
            odometry.twist.twist.angular.z = w_c;

            odometry.pose.pose.position.x = odometry.pose.pose.position.x + (odometry.twist.twist.linear.x * TIMESTEP);
            odometry.pose.pose.position.y = odometry.pose.pose.position.y + (odometry.twist.twist.linear.y * TIMESTEP);
            odometry.pose.pose.position.z = 0;
            odometry.pose.pose.orientation.x = 0;
            odometry.pose.pose.orientation.y = 0;
            odometry.pose.pose.orientation.z = odometry.pose.pose.orientation.z + (odometry.twist.twist.angular.z * TIMESTEP);
            odometry.pose.pose.orientation.w = 1;

            ROS_INFO_STREAM("\n" << odometry.pose.pose << odometry.twist.twist);

        }else{
            ROS_INFO_STREAM("Wrong mode. Please use a mode 1 or 2");
            }

        odom_pub.publish(odometry);
        
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 666;
}