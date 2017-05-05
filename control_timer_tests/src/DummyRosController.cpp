#include <DummyRosController.h>
#include <pluginlib/class_list_macros.h>

DummyRosController::~DummyRosController(){}

DummyRosController::DummyRosController(){}

DummyRosController::DummyRosController(DummyHardwareInterfacePtr iface){
    loadInterface(iface);
}

void DummyRosController::update(const ros::Time &, const ros::Duration &){
    std::cout << "RosController Update Starting" << std::endl;

    auto currentMsg = getMsg();

    for(auto letter : currentMsg){
        std::cout << letter;
        std::this_thread::sleep_for(getPerLetterDuration());
    }

    hardwareInterface->setMsg(currentMsg);

    std::cout << std::endl << "RosController Update Done" << std::endl;

    //return true;
}


PLUGINLIB_EXPORT_CLASS(DummyRosController, controller_interface::ControllerBase)
