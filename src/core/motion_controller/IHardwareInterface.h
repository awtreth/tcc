#ifndef IHARDWARE_INTERFACE_H
#define IHARDWARE_INTERFACE_H

class IHardwareInterface {

    public:
    virtual bool write() = 0;

    virtual bool read() = 0;
};

#endif
