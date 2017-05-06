#include <controller_interface/controller.h>
#include <DummyHardwareInterface.h>
#include <DummyControllerBase.h>
#include <joint_state_controller/joint_state_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <thread>

using namespace hardware_interface;

class DummyRosController : public DummyControllerBase,
        public controller_interface::Controller<DummyHardwareInterface>{

    // ControllerBase interface
public:

//    virtual ~DummyRosController();

    DummyRosController();

    DummyRosController(DummyHardwareInterface *iface);

    void update(const ros::Time&, const ros::Duration&);


    // ControllerBase interface
public:
    void starting(const ros::Time &){}
    void stopping(const ros::Time &){}

    // Controller interface
public:
    bool init(DummyHardwareInterface* iface, ros::NodeHandle &);
};

