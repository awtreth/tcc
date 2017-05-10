#include <DummyRosController.h>
#include <pluginlib/class_list_macros.h>

//DummyRosController::~DummyRosController(){}

DummyRosController::DummyRosController(){}

DummyRosController::DummyRosController(DummyHardwareInterface* iface){
    loadInterface(iface);
}

void DummyRosController::update(const ros::Time &, const ros::Duration &){
    std::cout << "RosController Update Starting" << std::endl;

    auto currentMsg = getMsg();

    std::cout << "Configurou mensagem" << std::endl;

    for(auto letter : currentMsg){
        std::cout << letter;
        std::this_thread::sleep_for(getPerLetterDuration());
    }

    std::cout << "Exibiu msg" << std::endl;

    hardwareInterface->setMsg(currentMsg);

    std::cout << std::endl << "RosController Update Done" << std::endl;

    //return true;
}

bool DummyRosController::init(DummyHardwareInterface * iface, ros::NodeHandle &){
    loadInterface(iface);

    return true;

}

PLUGINLIB_EXPORT_CLASS(DummyRosController, controller_interface::ControllerBase)
