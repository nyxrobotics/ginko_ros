#ifndef futaba_DRIVER
#define futaba_DRIVER

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nodelet/nodelet.h>
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <unistd.h>
#include <cstdlib>

#include "serial.hpp"

class futaba_driver {
    private:
    int count;
    double torque_const[30];
    // the number of motors may not exceed over 30, I guess.
    ros::NodeHandle nh;
    ros::Publisher present_states_pub;
    ros::Subscriber goal_states_sub;
    void init(ros::NodeHandle);
    void write_position(int*);
    bool get_status(std::vector<double>*, std::vector<double>*);
    boost::shared_ptr<serial_port> sp;

    public:
    futaba_driver(ros::NodeHandle, ros::NodeHandle);
    ~futaba_driver();
    void switch_torque(unsigned char, bool);
    void servo_reboot(unsigned char);
    void update_position(const sensor_msgs::JointState::ConstPtr&);
    void publish_status();
    void ginko_usleep(int usec);
    void set_baud_115200(void);
    void set_baud_230400(void);
    void set_baud_460800(void);
};

namespace ginko_joint {
    class revolute : public nodelet::Nodelet {
        private:
        boost::shared_ptr<futaba_driver> od;

        public:
        revolute();
        ~revolute();
        virtual void onInit();
    };
}

#endif 
