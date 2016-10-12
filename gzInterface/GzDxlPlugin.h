#ifndef SIMPLEARMPLUGIN_H
#define SIMPLEARMPLUGIN_H


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>
#include <gz_interface_msgs.pb.h>

namespace gazebo
{
    typedef const boost::shared_ptr<
    const gz_interface_msgs::msg::GzDxlRequest>
    GzDxlRequestPtr;

    class GzDxlPlugin :public ModelPlugin
    {
        private:

        /// \brief Plugin node
        private: transport::NodePtr node;

        /// \brief A subscriber to input commands.
        private: transport::SubscriberPtr cmdSub;

        /// \brief A publisher to broadcast model status.
        private: transport::PublisherPtr statusPub;

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joints (std::vector<JointPtr>)
        private: physics::Joint_V joints;

        /// \brief Pointer to the JointController
        private: physics::JointControllerPtr jointController;

        public:

        public: GzDxlPlugin();

        public: void Load(physics::ModelPtr _world, sdf::ElementPtr _sdf);

        private: void HandleCommand(GzDxlRequestPtr &_msg);

        private: void SetPositions(GzDxlRequestPtr &_msg);

        private: void SetVelocities(GzDxlRequestPtr &_msg);

        private: void SetTorques(GzDxlRequestPtr &_msg);

        //private: void GetPositions();

        //private: void GetVelocities();

        //private: void GetTorques();

    };
    GZ_REGISTER_MODEL_PLUGIN(GzDxlPlugin);
}

#endif // SIMPLEARMPLUGIN_H
