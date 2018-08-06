#include <stdio.h>	/* for printf() */
#include <stdlib.h>
#include <unistd.h>

#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <ros/package.h>

#include <comedilib.h>
#include <comedi.h>
#include <sensor.h>
#include <ftconfig.h>
#include <ctime>
#include <kcl_grasp_msgs/KCL_Sensor_Bias.h>


//write to file
#include <fstream>

using namespace std;


class DAQ_Sensor{

public:
    DAQ_Sensor();
    ~DAQ_Sensor(){}
    double rate;

    int init_comedi();
    int run();
    bool sensorbias(kcl_grasp_msgs::KCL_Sensor_Bias::Request& req,kcl_grasp_msgs::KCL_Sensor_Bias::Response& response);
    Calibration *cal;

private:

    int subdev;		/* change this to your input subdevice */
    //    int chan = 0;		/* change this to your channel */
    int range;		/* more on this later */
    int aref;/* more on this later */
    float voltages[NUM_CHAN];
    int flags;

    comedi_t *it;
    int chan;
    lsampl_t data;
    int retval;
    comedi_polynomial_t converter;
    float result[NUM_CHAN];


    ros::NodeHandle n;
    geometry_msgs::WrenchStamped sensor;
    ros::Publisher sensor_pub;
    ros::ServiceServer biasservice;

    std::string calfile;
    std::string daqdevice;
    char calfilepath[1024];

    int get_converter(comedi_t *device, unsigned subdevice, unsigned channel,
                      unsigned range, comedi_polynomial_t *converter);
};

DAQ_Sensor::DAQ_Sensor(){

    subdev = 0;		/* change this to your input subdevice */
    //    int chan = 0;		/* change this to your channel */
    range = 0;		/* more on this later */
    aref = AREF_GROUND;	/* more on this later */
    chan=0;
    if(!n.getParam("sensor/calibration", calfile) || !n.getParam("sensor/device", daqdevice) || !n.getParam("sensor/rate", rate)){
        ROS_ERROR("Unable to load the sensor parameters");
    }


    sensor_pub=n.advertise<geometry_msgs::WrenchStamped>("nano17ft",10);
    biasservice= n.advertiseService("sensor_bias", &DAQ_Sensor::sensorbias, this);




sensor.header.frame_id=ros::this_node::getNamespace().substr(1);
}

bool DAQ_Sensor::sensorbias(kcl_grasp_msgs::KCL_Sensor_Bias::Request& req,kcl_grasp_msgs::KCL_Sensor_Bias::Response& response){
    ROS_INFO("Biasing %s", ros::this_node::getName().c_str());
    Bias(cal,voltages);
    response.reply=true;
    return true;
}


/* figure out if we are talking to a hardware-calibrated or software-calibrated board,
    then obtain a comedi_polynomial_t which can be used with comedi_to_physical */
int DAQ_Sensor::get_converter(comedi_t *device, unsigned subdevice, unsigned channel,
                  unsigned range, comedi_polynomial_t *converter)
{

    flags = comedi_get_subdevice_flags(device, subdevice);
    if(flags < 0)
    {
        comedi_perror("comedi_get_subdevice_flags");
        return -1;
    }


//       flags=0; //CHANGE ME!!!!! TO BYPASS SOFTWARE CALIBRATION


    if(flags & SDF_SOFT_CALIBRATED) // board uses software calibration
    {
        char *calibration_file_path = comedi_get_default_calibration_path(device);


        //parse a calibration file which was produced by the comedi_soft_calibrate program
        comedi_calibration_t* parsed_calibration =
                comedi_parse_calibration_file(calibration_file_path);
        free(calibration_file_path);
        if(parsed_calibration == NULL)
        {
            printf("calibration_file_path: %s",calibration_file_path);
            comedi_perror("comedi_parse_calibration_file");
            return -1;
        }

        // get the comedi_polynomial_t for the subdevice/channel/range we are interested in
        retval = comedi_get_softcal_converter(subdevice, channel, range,
                                              COMEDI_TO_PHYSICAL, parsed_calibration, converter);
        comedi_cleanup_calibration(parsed_calibration);


        if(retval < 0)
        {
            comedi_perror("comedi_get_softcal_converter");
            return -1;
        }

    }else // board uses hardware calibration
    {
        retval = comedi_get_hardcal_converter(device, subdevice, channel, range,
                                              COMEDI_TO_PHYSICAL, converter);
        if(retval < 0)
        {
            comedi_perror("comedi_get_hardcal_converter");
            return -1;
        }
    }

    return 0;
}


int DAQ_Sensor::init_comedi(){
    it = comedi_open(daqdevice.c_str());
    if(it == NULL || it<0)
    {
        ROS_ERROR("Is the comedi installed? Can this user access the device (sudo chown <user> /dev/comedi*)?");
        comedi_perror("comedi_open");
        return -1;
    }

    /*First reading and Biasing */

    for (chan=0;chan<NUM_CHAN; chan++){
        retval = comedi_data_read(it, subdev, chan, range, aref, &data);

        if(retval < 0)	{
            comedi_perror("comedi_data_read");
            return -1;
        }

        retval = get_converter(it, subdev, chan, range, &converter);

        if(retval < 0)	{
            return -1;
        }

        voltages[chan] = comedi_to_physical(data, &converter);
    }

    std::string path = ros::package::getPath("kcl_grasp_tactile");

    strcpy(calfilepath,path.c_str());
    strcat(calfilepath,calfile.c_str());

    cal=createCalibration(calfilepath,1);
    SetForceUnits(cal,(char*) "N");
    SetTorqueUnits(cal,(char*) "N-mm");
    Bias(cal,voltages);
}


int DAQ_Sensor::run(){


    for (chan=0;chan<NUM_CHAN; chan++){
        retval = comedi_data_read(it, subdev, chan, range, aref, &data);

        if(retval < 0)	{
            comedi_perror("comedi_data_read");
            return -1;
        }

  //      retval = get_converter(it, subdev, chan, range, &converter);
        //if(retval < 0)	return -1;

        voltages[chan]= comedi_to_physical(data, &converter);
    }

    ConvertToFT(cal,voltages,result);


ROS_DEBUG_STREAM_THROTTLE(1,ros::Time::now().toSec() << "," <<voltages[0] << ","<< voltages[1] << ","<< voltages[2] << ","<< voltages[3] << ","<< voltages[4] << ","<< voltages[5] << ","<< result[0] << "," << result[1] << "," << result[2] << "," << result[3] << "," << result[4] << "," << result[5] << std::endl);


    sensor.wrench.force.x=result[0];
    sensor.wrench.force.y=result[1];
    sensor.wrench.force.z=result[2];
    sensor.wrench.torque.x=result[3];
    sensor.wrench.torque.y=result[4];
    sensor.wrench.torque.z=result[5];
    sensor.header.stamp=ros::Time::now();

    sensor_pub.publish(sensor);
    return 0;
}

int main(int argc,char *argv[])
{

    ros::init(argc, argv, "sensor");

    DAQ_Sensor daq_sensor;
    ros::Rate loop_rate(daq_sensor.rate);

    //Comedi Stuff

    if(daq_sensor.init_comedi()!=0) {
        ROS_ERROR("Error in init_comedi");
        return -1;
}

    /* Start Readings */


    while (ros::ok()) {

        if(daq_sensor.run()!=0){
            ROS_ERROR("Error running");
        }
        //    ROS_DEBUG("[%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f]", sensor.wrench.force.x, sensor.wrench.force.y, sensor.wrench.force.z,sensor.wrench.torque.x, sensor.wrench.torque.y, sensor.wrench.torque.z);

        loop_rate.sleep();
        ros::spinOnce();
    }

    destroyCalibration(daq_sensor.cal);
    return 0;
}

