#include <DummyController.h>


DummyController::DummyController()
{

}

DummyController::DummyController(DummyHardwareInterface* iface){
    loadInterface(iface);
}

bool DummyController::prepareRead(std::chrono::steady_clock::time_point){ return true;}

bool DummyController::update(std::chrono::steady_clock::time_point){
    std::cout << "Update Starting" << std::endl;

    auto currentMsg = getMsg();

    for(auto letter : currentMsg){
        std::cout << letter << std::flush;
        std::this_thread::sleep_for(getPerLetterDuration());
    }

    hardwareInterface->setMsg(currentMsg);

    std::cout << std::endl << "Update Done" << std::endl;

    return true;
}
