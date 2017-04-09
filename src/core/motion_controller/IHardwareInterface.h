#ifndef IHARDWARE_INTERFACE_H
#define IHARDWARE_INTERFACE_H

class IHardwareInterface {

    bool write() = 0;

    bool read() = 0;
};

#endif
