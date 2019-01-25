#pragma once
#include "ros_envelope_iface.h"
#include "ros_envelope_iface_impl.h"
#include "ros_envelope_template_impl.h"
#include <memory>

namespace RACE {
namespace ROS {

/**
 * @brief The RosEnvolope class contains a single ROS message of type <msg_type>
 * ROS messages can be transfered between envelopes without the concrete version of the message being known to the client
 * Inherrits from a base class so that it can be put in a vector
 */
template<class ros_msg>
class RosEnvelope : public RosEnvelopeInterface
{
public:

    explicit RosEnvelope(ros_msg msg,std::string topicName, int pubQueueSize, ros::NodeHandle& n)
    {
         //Set the implementation of this class to the RosEnvolopeInterfaceImplTemplate<msg_type> implementation
        this->pimpl = new RosEnvelopeInterfaceImplTemplate<ros_msg>(msg, topicName,pubQueueSize,n);
           //     std::make_shared<RosEnvelopeInterfaceImplTemplate<ros_msg> >();
    }

    explicit RosEnvelope()
    {
         //Set the implementation of this class to the RosEnvolopeInterfaceImplTemplate<msg_type> implementation
        this->pimpl = new RosEnvelopeInterfaceImplTemplate<ros_msg>;
           //     std::make_shared<RosEnvelopeInterfaceImplTemplate<ros_msg> >();
    }

    virtual ~RosEnvelope()
    {
        delete this->pimpl;
    }

    ros_msg getContents()
    {
        //The cast is safe because the base type Impl* can never be constructed - only its subclass RosEnvolopeInterfaceImplTemplate* can
        RosEnvelopeInterfaceImplTemplate<ros_msg>* castedPimpl = static_cast<RosEnvelopeInterfaceImplTemplate<ros_msg>* >(this->pimpl);
        return castedPimpl->getContents();

    }

};

}//namespace ROS
}//namespace RACE
