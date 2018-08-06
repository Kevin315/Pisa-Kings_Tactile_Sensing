#include <cstdio>
#include <sr_grasp_msgs/KCL_ContactStateStamped.h>
#include <sr_robot_msgs/ShadowPST.h>
#include <ros/ros.h>

using sr_grasp_msgs::KCL_ContactStateStamped;
using sr_robot_msgs::ShadowPST;
using namespace std;

#define N_FINGERS 5

class PST_Sensors
{
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_pst_;

  // sample the first few values to establish a bias
  // it's in initializer list and it counts down
  int sample_size_;

  // same order of fingers in finger_used, bias, pubs, kcl_states and ShadowPST message
  bool finger_used_[N_FINGERS];
  double bias_[N_FINGERS];

  ros::Publisher pubs_[N_FINGERS];
  KCL_ContactStateStamped kcl_states_[N_FINGERS];

  void PST_callback(const ShadowPST &msg);

public:
  PST_Sensors();
  bool run();
};

PST_Sensors::PST_Sensors() :
  sub_pst_(nh_.subscribe("tactile", 10, &PST_Sensors::PST_callback, this)),
  sample_size_(10),
  finger_used_(),
  pubs_(),
  kcl_states_()
{
  string finger_names[N_FINGERS] = {"ffdistal",
                                    "mfdistal",
                                    "rfdistal",
                                    "lfdistal",
                                    "thdistal"};

  ros::NodeHandle nh_tilde("~");

  for (int i = 0; i < N_FINGERS; ++i)
  {
    nh_tilde.getParam(finger_names[i], finger_used_[i]);
    if (finger_used_[i])
    {
      pubs_[i] = nh_.advertise<KCL_ContactStateStamped>(finger_names[i] + "/ContactState", 10);

      kcl_states_[i].header.frame_id = finger_names[i];
      kcl_states_[i].contact_position.z = 0.016;
      kcl_states_[i].contact_position.y = -0.006;
      kcl_states_[i].contact_normal.y = -1.0;

      bias_[i] = (double) INT_MAX;
    }
  }
}

void PST_Sensors::PST_callback(const ShadowPST &msg)
{
  // Work out the bias
  if (sample_size_)
  {
    for (int i = 0; i < N_FINGERS; ++i)
    {
      if (msg.pressure[i] < bias_[i])
      {
        bias_[i] = msg.pressure[i];
      }
    }
    --sample_size_;
    return;
  }

  // Calculate force
  for (int i = 0; i < N_FINGERS; ++i)
  {
    if (finger_used_[i])
    {
      kcl_states_[i].header.stamp = ros::Time::now();
      kcl_states_[i].Fnormal = -(msg.pressure[i] - bias_[i]) / 100.0;
      pubs_[i].publish(kcl_states_[i]);
    }
  }
}

bool PST_Sensors::run()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pst_kcl_sensor");

  PST_Sensors pst_sensor;
  pst_sensor.run();

  return 0;
}
