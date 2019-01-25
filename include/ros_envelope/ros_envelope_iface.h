#pragma once
#include <string>
#include "ros/ros.h"

namespace RACE {
namespace ROS {



/**
 * @brief The RosEnvelopeInterface class is the base class for a templated ROS Envelope that can store a ROS message
 */
class RosEnvelopeInterface
{
public:

    explicit RosEnvelopeInterface() {}
    virtual ~RosEnvelopeInterface() {}

    //Copy and store a message from another envelope
    void addMessage(RosEnvelopeInterface* srcEnvelope);

    //Advertise the contained message
    void advertise(std::string topicName, int pubQueueSize, ros::NodeHandle& n);

    //Publish the contained message
    void publish();

    //Checj if the envelope contains a message
    bool isEmpty();

public:
    //Forward declare this classes implementation
    class Impl;
    //Pointer to this classes implementation
    Impl* pimpl;

public:
    Impl *getPimpl() const;

};

}//namespace ROS
}//namespace RACE
