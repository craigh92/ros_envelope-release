#pragma once
#include "ros_envelope_iface_impl.h"
#include "ros/ros.h"
#include "ros/console.h"

namespace RACE {
namespace ROS {

/**
 * @brief The RosEnvolope class contains a single ROS message of type <msg_type>
 * ROS messages can be transfered between envelopes without the concrete version of the message being known to the client
 * Inherrits from a base class so that it can be put in a vector
 */
template<class ros_msg>
class RosEnvelopeInterfaceImplTemplate : public RosEnvelopeInterface::Impl
{
public:

    explicit RosEnvelopeInterfaceImplTemplate(ros_msg msg,std::string topicName, int pubQueueSize, ros::NodeHandle &n)
    {
        this->topicName = topicName;
        this->pubQueueSize = pubQueueSize;
        this->n = n;
        this->contents = msg;
        this->_isEmpty = false;
    }

    explicit RosEnvelopeInterfaceImplTemplate(ros_msg msg)
    {
        this->contents = msg;
        this->_isEmpty = false;
    }

    explicit RosEnvelopeInterfaceImplTemplate()
    {
        this->_isEmpty = true;
    }

    virtual ~RosEnvelopeInterfaceImplTemplate() {}

    /**
     * @brief addMessage - Transfers a ROS message from one envolope to the next without the concrete version of the ROS message being known to the client
     * @param msg - The ROS Envolope containing the desired message
     * @return
     */
    virtual void addMessage(RosEnvelopeInterface* srcEnvelope) final
    {        
        //The cast is safe because the base type RosEnvolopeInterface* can never be constructed - only its subclass RosEnvolopeInterfaceImplTemplate* can
        RosEnvelopeInterface::Impl* srcEnvelopePimpl = srcEnvelope->getPimpl();
        RosEnvelopeInterfaceImplTemplate<ros_msg>* castedSrcEnvelopePimpl = NULL;
        castedSrcEnvelopePimpl = static_cast<RosEnvelopeInterfaceImplTemplate<ros_msg>*>(srcEnvelopePimpl);
        if(castedSrcEnvelopePimpl == NULL) std::cout << "Failed to cast!" << std::endl;

        //Put the contents of the src envolope into this envolope
        this->contents = castedSrcEnvelopePimpl->getContents();

        //Copy the empty/not empty status across
        this->_isEmpty = castedSrcEnvelopePimpl->isEmpty();
    }

    virtual void publish()
    {
        ROS_DEBUG_STREAM("Publishing message");
        pub.publish(contents);
        ROS_DEBUG_STREAM("Message published");
    }

    ros_msg getContents() const
    {
        return contents;
    }

    virtual bool isEmpty()
    {
        return _isEmpty;
    }

    void advertise(std::string topicName, int pubQueueSize, ros::NodeHandle& n)
    {
        ROS_DEBUG_STREAM("Creating publisher");
        pub = n.advertise<ros_msg>(topicName, pubQueueSize);
        ROS_DEBUG_STREAM("Publisher created");
    }

    ros_msg contents;
    ros::Publisher pub;
    bool _isEmpty;

private:
    std::string topicName;
    int pubQueueSize;
    ros::NodeHandle n;
};



}//namespace ROS
}//namespace RACE
