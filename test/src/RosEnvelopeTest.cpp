#include <iostream>
#include <ros_envelope/ros_envelope.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gtest/gtest.h>

using namespace RACE::ROS;

TEST(ros_envelope_tests, ctor_dtor_test)
{
    ros::NodeHandle nh("~");

        std::cout << "Construction/Destruction" << std::endl;

        {
            try
            {
                RosEnvelope<sensor_msgs::Imu> rosEnvelope();
                std::cout << "\tPass!" << std::endl;
            }
            catch(...)
            {
                std::cout << "\tFail!" << std::endl;
                EXPECT_TRUE(false);
            }
        }


        {
            try
            {
                sensor_msgs::Imu imuMsg;
                RosEnvelope<sensor_msgs::Imu> rosEnvelope(imuMsg,"IMU",1000,nh);
                std::cout << "\tPass!" << std::endl;
            }
            catch(...)
            {
                std::cout << "\tFail!" << std::endl;
                EXPECT_TRUE(false);
            }
        }
}

TEST(ros_envelope_tests, interface_test)
{
    ros::NodeHandle nh("~");

        std::cout << "Interface Test" << std::endl;

        {
            try
            {
                sensor_msgs::Imu imuMsg;
                RosEnvelope<sensor_msgs::Imu>* envolope = new RosEnvelope<sensor_msgs::Imu>(imuMsg,"IMU",1000,nh);
                RosEnvelopeInterface* envolopeInterface = envolope;
                delete envolope;
                std::cout << "\tPass!" << std::endl;

            }
            catch(...)
            {
                std::cout << "\tFail!" << std::endl;
                EXPECT_TRUE(false);
            }
        }
}

TEST(ros_envelope_tests, transfer_test)
{
    ros::NodeHandle nh("~");

        std::cout << "Envelope Transfer Test" << std::endl;

        {
            sensor_msgs::Imu imuMsg;
            imuMsg.orientation.x = 3;
            RosEnvelope<sensor_msgs::Imu> srcEnvolope(imuMsg,"IMU",1000,nh);
            if(srcEnvolope.isEmpty())
            {
                std::cout << "\tFail!" << std::endl;
                EXPECT_TRUE(false);
            }
            RosEnvelope<sensor_msgs::Imu> destEnvolope;
            if(!destEnvolope.isEmpty())
            {
                std::cout << "\tFail!" << std::endl;
                EXPECT_TRUE(false);
            }
            destEnvolope.addMessage(&srcEnvolope);
            if(destEnvolope.isEmpty())
            {
                std::cout << "\tFail!" << std::endl;
                EXPECT_TRUE(false);
            }

            std::cout << "\t\tOpen Test" << std::endl;

            {
                sensor_msgs::Imu receievedMsg = destEnvolope.getContents();
                float x = receievedMsg.orientation.x;
                if(x != 3)
                {
                    std::cout << "\t\t\tFail! " << std::endl;
                    EXPECT_TRUE(false);
                }
                else std::cout << "\t\t\tPass!" << std::endl;
            }

            std::cout << "\tPass!" << std::endl;

        }
}

TEST(ros_envelope_tests, vector_test)
{
    ros::NodeHandle nh("~");

        std::cout << "Vector Test" << std::endl;

        {
            sensor_msgs::Imu imuMsg;
            imuMsg.orientation.x = 5;
            geometry_msgs::Wrench loadMsg;
            loadMsg.force.y = 9;

            RosEnvelopeInterface* env_1 = new RosEnvelope<sensor_msgs::Imu>(imuMsg,"IMU",1000,nh);
            RosEnvelopeInterface* env_2 = new RosEnvelope<geometry_msgs::Wrench>(loadMsg,"Load",1000,nh);

            std::vector<RosEnvelopeInterface*> mailBag;
            mailBag.push_back(env_1);
            mailBag.push_back(env_2);

            RosEnvelope<sensor_msgs::Imu> destImuEnvolope;
            RosEnvelope<geometry_msgs::Wrench> destLoadEnvolope;

            destImuEnvolope.addMessage(mailBag.at(0));
            destLoadEnvolope.addMessage(mailBag.at(1));

            sensor_msgs::Imu receivedImuMsg = destImuEnvolope.getContents();
            geometry_msgs::Wrench receivedLoadMsg = destLoadEnvolope.getContents();

            float x = receivedImuMsg.orientation.x;
            float y = receivedLoadMsg.force.y;

            if((x != 5) && (y != 9))
            {
                std::cout << "\tFail! " << std::endl;
                EXPECT_TRUE(false);
            }
            else std::cout << "\tPass!" << std::endl;
        }

        std::cout << "Done" << std::endl;
}

TEST(ros_envelope_tests, adv_pub_test)
{
    ros::NodeHandle nh("~");
    sensor_msgs::Imu imuMsg;
    imuMsg.orientation.x = 8.123;
    RosEnvelopeInterface* env_1 = new RosEnvelope<sensor_msgs::Imu>(imuMsg,"IMU",1000,nh);
    RosEnvelope<sensor_msgs::Imu> destImuEnvolope;
    destImuEnvolope.addMessage(env_1);
    try
    {
        destImuEnvolope.advertise("IMU",1000,nh);
    }
    catch(...)
    {
        std::cout << "\tFail! " << std::endl;
        EXPECT_TRUE(false);
    }
    try
    {
        destImuEnvolope.publish();
    }
    catch(...)
    {
        std::cout << "\tFail! " << std::endl;
        EXPECT_TRUE(false);
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "tester");
   ros::NodeHandle nh("~");
   return RUN_ALL_TESTS();
}
