/// \file
/// \brief A brief description of what the file does
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     topic_name (topic_type): description of topic
/// SUBSCRIBES:
///     topic_name (topic_type): description of the topic
/// SERVICES:
///     service_name (service_type): description of the service

#include <iostream>
#include "ros/ros.h"

/// Static variables used by callbacks here

enum class State {STOP, GO, END};

static State state = State::STOP;

void stop_callback(const MessageType & msg)
{
    // process message.  Maybe messages of this type
    // mean its supposed to go into STOP mode
    state = State::STOP;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;

    // read parameters, create publishers/subscribers
    const auto sub = nh.subscribe("stop", 1000, stop_callback);

    ros::Rate r(frequency)
    while(ros::ok())
    {
        switch(s)
        {   
            case State::STOP:
                // do stop state stuff
                break;
            case State::GO:
                // do go state stuff
                break;
            case State::END:
                // do end state stuff
                break;
            default:
                // should never get here
                throw std::logic_error("Invalid State");
        }
        ros::spinOnce();
        r.sleep()
    }
    return 0;   
}