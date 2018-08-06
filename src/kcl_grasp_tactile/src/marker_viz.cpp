#include <cstdio>
#include <math.h>
#include <sys/time.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <sr_grasp_msgs/KCL_ContactStateStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/ColorRGBA.h>
#include <XmlRpcValue.h>
#include <ros/xmlrpc_manager.h>


class Arrow{
public:
    ros::NodeHandle nh;
    Arrow(ros::NodeHandle);
private:
    visualization_msgs::Marker marker;
    std::vector<ros::Subscriber> subscribers;
    ros::Publisher marker_pub;
    void MarkerCB(const sr_grasp_msgs::KCL_ContactStateStamped);
    std::vector<std::string> finger_names;
    std::map<std::string,std::string> frame_names;
    std_msgs::ColorRGBA c_green,c_red;
};

Arrow::Arrow(ros::NodeHandle n) {

    nh=n;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type=visualization_msgs::Marker::ARROW;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.002;
    marker.scale.y = 0.005;
    marker.scale.z = 0.0;

    // Set the color -- be sure to set alpha to something non-zero!
    c_green.a=0.5;
    c_green.r=0.0;
    c_green.g=1.0;
    c_green.b=0.0;
    c_red.a=0.5;
    c_red.r=1.0;
    c_red.g=0.0;
    c_red.b=0.0;
    marker.color=c_green;

    marker.points.resize(2);

    XmlRpc::XmlRpcValue my_list;
    nh.getParam("fingers", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::string frame;

    for (int32_t i = 0; i < my_list.size(); ++i)
    {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        finger_names.push_back(static_cast<std::string>(my_list[i]));
        subscribers.push_back(nh.subscribe(finger_names[i] + "/ContactState",10,&Arrow::MarkerCB,this));
        nh.getParam(finger_names[i] + "/frame", frame);
        frame_names[finger_names[i]]=ros::this_node::getNamespace().substr(1) + "/" + frame;
        std::cout << "Frame: " << frame << " | " << finger_names[i] <<  " #  [" << finger_names[i] << "]:"  <<  frame_names[finger_names[i]] << std::endl;
    }


    marker_pub=nh.advertise<visualization_msgs::Marker>("force_markers",10);


}

void Arrow::MarkerCB(const sr_grasp_msgs::KCL_ContactStateStamped msg){
    geometry_msgs::Point p;
    p=msg.contact_position;
    p.x-=(msg.contact_normal.x*msg.Fnormal+msg.tangential_force.x)*0.01;
    p.y-=(msg.contact_normal.y*msg.Fnormal+msg.tangential_force.y)*0.01;
    p.z-=(msg.contact_normal.z*msg.Fnormal+msg.tangential_force.z)*0.01;


    marker.points.at(0)=p;
    marker.points.at(1)=msg.contact_position;

    //marker.header.stamp=ros::Time::now();
    marker.header.frame_id=msg.header.frame_id;
    marker.header.stamp=msg.header.stamp;
    marker.ns=msg.header.frame_id;
    marker.color=c_green;
    marker_pub.publish(marker);


    p=msg.contact_position;
    p.x+=msg.contact_normal.x*0.01;
    p.y+=msg.contact_normal.y*0.01;
    p.z+=msg.contact_normal.z*0.01;

    marker.points.at(0)=p;
    marker.color=c_red;
    marker.ns=msg.header.frame_id + "_n";
    marker_pub.publish(marker);

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "marker_viz");
  ros::NodeHandle nh;
  Arrow arrow(nh);
  ros::Rate loop_rate(10);

  while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
  }

}
