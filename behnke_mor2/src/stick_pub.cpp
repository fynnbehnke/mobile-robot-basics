#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Diese Node dient nur dazu die Stange aus der Gazebo Simulation als Marker zu publishen um sie anschließend in Rviz zu visualisieren.
// Dieses Topic wird gelatched, da die Daten sich nicht verändern und somit einmal publishen ausreichend ist.

int main(int argc, char** argv){
    
    ros::init(argc, argv, "stick_pub_node");

    ros::NodeHandle nh;
    ros::Publisher stick_marker_pub = nh.advertise<visualization_msgs::Marker>("/stick_marker", 1, true);

    visualization_msgs::Marker gazebo_stick;

    gazebo_stick.header.frame_id = "/odom";
    gazebo_stick.id = 0;
    gazebo_stick.type = visualization_msgs::Marker::CYLINDER;
    gazebo_stick.pose.position.x = 4;
    gazebo_stick.pose.position.y = -2;
    gazebo_stick.pose.position.z = 1;
    gazebo_stick.pose.orientation.x = 0.0;
    gazebo_stick.pose.orientation.y = 0.0;
    gazebo_stick.pose.orientation.z = 0.0;
    gazebo_stick.pose.orientation.w = 1.0;
    gazebo_stick.scale.x = 0.1;
    gazebo_stick.scale.y = 0.1;
    gazebo_stick.scale.z = 2.0;
    gazebo_stick.color.r = 0.0;
    gazebo_stick.color.g = 0.0;
    gazebo_stick.color.b = 1.0;
    gazebo_stick.color.a = 1.0;

    stick_marker_pub.publish(gazebo_stick);

    while(ros::ok()){
        ros::spin();
    }

    return 666; // Copyright by Fynn Behnke - mr18b070 😈
}
