#pragma once
#include "iostream"
#include "ros_envelope_iface.h"
#include "ros/ros.h"
#include "ros/console.h"

namespace RACE {
namespace ROS {

/**
* @brief An interface to an implementation of the RosEnvelope.
* A ROS Envolope can store a ROS message of any type. (Similar to how a std::vector can store any type)
*
* This implementation of the ROS Envelope can not contain a ROS message.
* Since the overide of the addMessage function uses a cast to the RosEnvelopeTemplate class,
* we need to ensure that an instance of this class is never constructed.
*/
class RosEnvelopeInterface::Impl
{
public:

    explicit Impl() {}

public:

    virtual ~Impl() {}

public:

    virtual void addMessage(RosEnvelopeInterface* srcEnvolope) =0;

    virtual void advertise(std::string topicName, int pubQueueSize, ros::NodeHandle& n)=0;

    virtual void publish()=0;

    virtual bool isEmpty()=0;

};

}//namespace ROS
}//namespace RACE
