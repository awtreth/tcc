#include <iostream>
#include <ControlTimer.h>
#include <DummyController.h>
#include <DummyHardwareInterface.h>
#include <stdio.h>
#include <memory>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

int main(){

    auto interface = std::make_shared<DummyHardwareInterface>();

    auto controller = std::make_shared<DummyController>(interface.get());

    controller->setMsg("MENSAGEM MAIOR");
    controller->setPerLetterDuration(microseconds(long(100e3)));

    ControlTimer controlTimer(interface);

    controlTimer.setFrequency(10);

    controlTimer.loadController("DummyController",controller);

    controlTimer.resumeLoop();

    sleep(1000);

	return 0;
}
