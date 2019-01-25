#include "ros_envelope/ros_envelope_iface.h"
#include "ros_envelope/ros_envelope_iface_impl.h"
using namespace RACE::ROS;

//Copy and store a message from another envelope
void RosEnvelopeInterface::addMessage(RosEnvelopeInterface* srcEnvolope)
{
    this->pimpl->addMessage(srcEnvolope);
}

void RosEnvelopeInterface::advertise(std::string topicName, int pubQueueSize, ros::NodeHandle& n)
{
    this->pimpl->advertise(topicName, pubQueueSize, n);
}

bool RosEnvelopeInterface::isEmpty()
{
    this->pimpl->isEmpty();
}

void RosEnvelopeInterface::publish()
{
    this->pimpl->publish();
}

RosEnvelopeInterface::Impl *RosEnvelopeInterface::getPimpl() const
{
    return pimpl;
}
