#include <ros/ros.h>
#include <std_msgs/String.h>
#include "dtransmit/dtransmit.hpp"

// TODO(MWX): test on different machine, over wifi
using namespace dtransmit;
using namespace std;
int main(int argc, char** argv)
{
//    DTransmit d("127.0.0.1");
    DTransmit d("192.168.255.255");

    int NUM = 10;

    for(int i = 0; i < NUM; ++i) {
        d.add_recv<std_msgs::String>(2000 + i, [=](std_msgs::String& msg) {
            ROS_INFO("%d heard: [%s]", 2000 + i, msg.data.c_str());
        });
    }

    d.startRecv();

    for(int i = 0 ; i < 100 ; ++i) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello dtransmit " << i;
        msg.data = ss.str();

        ROS_INFO("Sending: [%s]", msg.data.c_str());

        for(int j = 0; j < NUM; ++j) {
            d.send<std_msgs::String>(2000 + j, msg);
        }
//        usleep(100);
        cout << endl;
    }
}
