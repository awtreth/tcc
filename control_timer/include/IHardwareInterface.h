#ifndef IHARDWARE_INTERFACE_H
#define IHARDWARE_INTERFACE_H

#include <memory>

using namespace std;

class IHardwareInterface {

    public:
    virtual bool write() = 0;

    virtual bool read() = 0;

    virtual ~IHardwareInterface(){}
};

typedef shared_ptr<IHardwareInterface> HardwareInterfacePtr;

#endif
