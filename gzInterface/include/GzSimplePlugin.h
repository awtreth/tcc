#ifndef GZSIMPLEPLUGIN_H
#define GZSIMPLEPLUGIN_H


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>
//#include <gz_interface_msgs.pb.h>

#include <GzWriteRequest.pb.h>
#include <GzReadRequest.pb.h>
#include <GzReadResponse.pb.h>

#define DEFAULT_GZ_WRITE_REQUEST_TOPIC_NAME "/simple_plugin/cmd"
#define DEFAULT_GZ_READ_ANSWER_TOPIC_NAME   "/simple_plugin/status"

namespace gazebo
{
    typedef const boost::shared_ptr<const gz_msgs::GzWriteRequest> GzWriteRequestPtr;

    typedef const boost::shared_ptr<const gz_msgs::GzReadRequest> GzReadRequestPtr;

    typedef const boost::shared_ptr<const gz_msgs::GzReadResponse> GzReadResponsePtr;

    class GzSimplePlugin :public ModelPlugin
    {
        private:

        transport::NodePtr node;

        transport::SubscriberPtr readRequestSub;

        transport::SubscriberPtr writeRequestSub;

        transport::PublisherPtr responsePub;

        physics::ModelPtr model;

        physics::Joint_V joints;

        physics::JointControllerPtr jointController;


        public:

        GzSimplePlugin();

        void Load(physics::ModelPtr _world, sdf::ElementPtr _sdf) override;

        void handleWriteRequest(GzWriteRequestPtr &msg);

        void handleReadRequest(GzReadRequestPtr &msg);

        void setPositions(GzWriteRequestPtr &_msg);

        void setVelocities(GzWriteRequestPtr &_msg);

        void setTorques(GzWriteRequestPtr &_msg);

        void setPosPids(GzWriteRequestPtr &msg);

        void setVelPids(GzWriteRequestPtr &msg);

    };
    GZ_REGISTER_MODEL_PLUGIN(GzSimplePlugin);
}

#endif // GZSIMPLEPLUGIN_H
