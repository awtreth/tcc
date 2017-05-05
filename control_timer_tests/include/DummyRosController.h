#include <controller_interface/controller.h>
#include <DummyHardwareInterface.h>
#include <DummyControllerBase.h>
#include <thread>

class DummyRosController : public DummyControllerBase, public controller_interface::Controller<DummyHardwareInterface>{

    // ControllerBase interface
public:

    virtual ~DummyRosController();

    DummyRosController();

    DummyRosController(DummyHardwareInterfacePtr iface);

    void update(const ros::Time&, const ros::Duration&);

};

