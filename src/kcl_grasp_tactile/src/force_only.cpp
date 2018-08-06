#include <cstdio>
#include <sr_grasp_msgs/KCL_ContactStateStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <Eigen/Core>

#define MAX 1000
#define pi 3.14159265

using namespace Eigen;

class ContactFO{

public:
    ContactFO();
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher KCL_ContactState_pub;

    void sensorCallback(const geometry_msgs::WrenchStamped&);
    int setFinger(char* arg);

protected:
    int finger;
    std::string frame_id_;
    double a,b,c,y00,z0,frot,sfrot,cfrot;


    geometry_msgs::Point changeFrame(geometry_msgs::Point in_pt);
    geometry_msgs::Vector3 changeFrame(geometry_msgs::Vector3 in_v);
    geometry_msgs::Vector3 normalise_vec(geometry_msgs::Vector3 in_v);
    geometry_msgs::Point contact_location(geometry_msgs::Vector3 normal);
    Vector4d func(Vector4d x, geometry_msgs::Vector3 normal);

};


ContactFO::ContactFO(){
    sub=n.subscribe("nano17ft", 100, &ContactFO::sensorCallback,this);
    KCL_ContactState_pub=n.advertise<sr_grasp_msgs::KCL_ContactStateStamped>("ContactState",10);
    n.getParam("frame", frame_id_);


    n.getParam("contact/a", a);
    n.getParam("contact/b", b);
    n.getParam("contact/c", c);
    n.getParam("contact/y00", y00);
    n.getParam("contact/z0", z0);
    n.getParam("contact/frot", frot);
    sfrot=sin(frot);
    cfrot=cos(frot);


}

geometry_msgs::Vector3 ContactFO::normalise_vec(geometry_msgs::Vector3 in_v){
    geometry_msgs::Vector3 out;
    double norm;

    norm=sqrt(in_v.x*in_v.x+in_v.y*in_v.y+in_v.z*in_v.z);
    out.x=in_v.x/norm;
    out.y=in_v.y/norm;
    out.z=in_v.z/norm;

    return out;
}

geometry_msgs::Point ContactFO::changeFrame(geometry_msgs::Point in_pt)
{
    geometry_msgs::Point new_pt;
    new_pt.x= (in_pt.x)/1000.0;
    new_pt.y= -in_pt.z/1000.0;
    new_pt.z= (in_pt.y+b)/1000.0;
    return new_pt;
}

//! changes frame from ellipsoid center to finger distal frame (used for shadow type finger)
geometry_msgs::Vector3 ContactFO::changeFrame(geometry_msgs::Vector3 in_v)
{
    geometry_msgs::Vector3 new_v;
    new_v.x= in_v.x;
    new_v.y= -in_v.z;
    new_v.z= in_v.y;
    return new_v;
}


Vector4d ContactFO::func(Vector4d x,geometry_msgs::Vector3 normal){
    Vector4d g;

    g[0]=(x[0]/a)*(x[0]/a) + ((x[1]-y00)/b)*((x[1]-y00)/b) + ((x[2]-z0)/c)*((x[2]-z0)/c) - 1;
    g[1]=normal.x+2/(a*a)*x[0];
    g[2]=normal.y+2/(b*b)*(x[1]-y00);
    g[3]=normal.z+2/(c*c)*(x[2]-z0);
    return g;
}


geometry_msgs::Point ContactFO::contact_location(geometry_msgs::Vector3 normal){

    geometry_msgs::Point cl;
    Vector4d g,jh,JtG,x,xh;
    Matrix4d J;

    g << 1,1,1,1;
    J.setZero(4,4);

    double h=0.02;
    double eps=2;
    int j=0;
    int i=0;

    x << 0,0,c+z0,0;


    int iter=0;

    while(fabs(g[0])>=0.01 || fabs(g[1])>=0.01 || fabs(g[2])>=0.01 || fabs(g[3])>=0.01)	{
        iter++;
        if(iter>MAX) break;

        g=func(x,normal);

        for(i=0;i<4;i++){
            xh=x;
            xh[i]+=h;
            jh=(func(xh,normal)-g)/h;
            for(j=0;j<4;j++) J(j,i)=jh(j);
        }
        JtG=J.transpose()*g;
        x=x-eps*JtG;
    //   std::cout << iter << ":\t" << x[0] << " "<< x[1] << " "<< x[2] << std::endl;

    }

    cl.x= x[0];
    cl.y= x[1];
    cl.z= x[2];


    //    cl.x= (a*a)/2 * normal.x;
    //    cl.y= (b*b)/2 * normal.y +y00;
    //    cl.z= (c*c)/2 * normal.z + z0;

  //  ROS_INFO("%f %f %f \t %f %f %f",normal.x,normal.y,normal.z,cl.x,cl.y,cl.z);


    return cl;

}

void ContactFO::sensorCallback(const geometry_msgs::WrenchStamped &msg){

    sr_grasp_msgs::KCL_ContactStateStamped contact;
    geometry_msgs::Vector3 in_v;

    in_v.x=cfrot*msg.wrench.force.x+sfrot*msg.wrench.force.y;
    in_v.y=-sfrot*msg.wrench.force.x+cfrot*msg.wrench.force.y;
    in_v.z=msg.wrench.force.z;



    contact.Fnormal=-sqrt(in_v.x*in_v.x+in_v.y*in_v.y+in_v.z*in_v.z);
    contact.contact_normal=changeFrame(normalise_vec(in_v));
    contact.contact_normal.x=-contact.contact_normal.x;
    contact.contact_normal.y=-contact.contact_normal.y;
    contact.contact_normal.z=-contact.contact_normal.z;
    contact.contact_position=changeFrame(contact_location(normalise_vec(in_v)));
    contact.header.stamp=msg.header.stamp;
    contact.header.frame_id=frame_id_;

    KCL_ContactState_pub.publish(contact);

}

int main(int argc,char *argv[]) {

    ros::init(argc, argv, "contact_force_only");
    ContactFO cloc;
    ros::NodeHandle n("~");
    cloc.n=n;


    while(ros::ok()){
        ros::spin();
    }



}
