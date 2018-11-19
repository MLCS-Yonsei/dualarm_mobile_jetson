#include "ros/ros.h"
#include <string>
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <imu_read_data/msgImu.h>


serial::Serial ser;

#define Pi 3.14159265359
/* Imu data */
double euler[3];
double gyro[3];
double acc[3];
double magnet[3];

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_read");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1, write_callback);

    /* Publisher and Subscriber */
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read",1);
    /* ros::Publisher imu_data_pub = nh.advertise<imu_read_data::msgImu>("/imu",100); */
    ros::Publisher toimu = nh.advertise<sensor_msgs::Imu>("/imu_data",10);
    ros::Publisher magnetic_pub = nh.advertise<sensor_msgs::MagneticField>("/mgn",10);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port Initialized");
    }else{
        return -1;
    }
    /* Publish rate : 100 Hz */
    ros::Rate loop_rate(30);

    while(ros::ok()){


        /* Declare variables */
        //imu_read_data::msgImu imu_data;
        sensor_msgs::Imu imu;
        sensor_msgs::MagneticField mgn;

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read:"<<result.data);
            read_pub.publish(result);



        /* the position to be chopped and the data to be sliced */
        std::size_t pos[11];
        std::string ch_data[12];



        /* The procedure of slicing */
        for(int i=0;i<11;i++){
            pos[i] = result.data.find(",");
            ch_data[i] = result.data.substr(1,pos[i]);
            result.data.replace(result.data.find(ch_data[i]),ch_data[i].length(),"");
        }
            ch_data[11]=result.data.substr(1,7);

        /* Convert string to float */
        for(int j=0;j<3;j++){
          euler[j]  = atof(ch_data[j].c_str());
          gyro[j]   = atof(ch_data[j+3].c_str());
          acc[j]    = atof(ch_data[j+6].c_str());
          magnet[j] = atof(ch_data[j+9].c_str());
        }

        /* eluer angle -unit conversion from deg to rad */
        euler[0] = euler[0] * Pi / 180;
        euler[1] = euler[1] * Pi / 180;
        euler[2] = euler[2] * Pi / 180;
        /* acceleration -unit conversion from g to m/s^2 */
        acc[0] = acc[0] * 9.81;
        acc[1] = acc[1] * 9.81;
        acc[2] = acc[2] * 9.81;
        /* gyro -unit conversion from deg/s to rad/s */
        gyro[0] = gyro[0] * Pi/180;
        gyro[1] = gyro[1] * Pi/180;
        gyro[2] = gyro[2] * Pi/180;

       /* Data Publish */
        //imu_data.euler_x = euler[0];
        //imu_data.euler_y = euler[1];
        //imu_data.euler_z = euler[2];

        //imu_data.gyro_x  = gyro[0];
        //imu_data.gyro_y  = gyro[1];
        //imu_data.gyro_z  = gyro[2];

        //imu_data.acc_x   = acc[0];
        //imu_data.acc_y   = acc[1];
        //imu_data.acc_z   = acc[2];

        //imu_data.magn_x  = magnet[0];
        //imu_data.magn_y  = magnet[1];
        //imu_data.magn_z  = magnet[2];

        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu_link";
        imu.linear_acceleration.x = acc[0];
        imu.linear_acceleration.y = acc[1];
        imu.linear_acceleration.z = acc[2];

        imu.angular_velocity.x = gyro[0];
        imu.angular_velocity.y = gyro[1];
        imu.angular_velocity.z = gyro[2];

        mgn.magnetic_field.x = magnet[0];
        mgn.magnetic_field.y = magnet[1];
        mgn.magnetic_field.z = magnet[2];

        double cy = cosf(euler[2] * 0.5);
        double sy = sinf(euler[2] * 0.5);
        double cr = cosf(euler[0] * 0.5);
        double sr = sinf(euler[0] * 0.5);
        double cp = cosf(euler[1] * 0.5);
        double sp = sinf(euler[1] * 0.5);

        imu.orientation.x = cy * sr * cp - sy * cr * sp;
        imu.orientation.y = cy * cr * sp + sy * sr * cp;
        imu.orientation.z = sy * cr * cp - cy * sr * sp;
        imu.orientation.w = cy * cr * cp + sy * sr * sp;

        /* imu_data_pub.publish(imu_data); */
        toimu.publish(imu);
        magnetic_pub.publish(mgn);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
