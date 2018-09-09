# Contact_location
Calculate contact location using Intrinsic Tactile Sensing method with Nano17 6-axis force/torque sensor data

**1. ATI force torque sensor driver: **
  A. Pci Card:
    roslaunch kcl_grasp_tactile single.launch
  B. Net box:
    rosrun netft_utils netft_node "IP address"
2. Contact location estimation:
  A. Iterative method:
      rosrun kcl_grasp_tactile contact_deform
  B. Integrated method (with closed form)
      rosrun kcl_grasp_tactile contact_loc_estimte
3. Visualization:
  rosrun kcl_grasp_tactile visualiser 
