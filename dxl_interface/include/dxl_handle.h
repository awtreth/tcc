#ifndef DXL_HANDLE_H
#define DXL_HANDLE_H

#include <dxl_model_spec.h>
#include <dxl_control_table.h>

namespace dynamixel {

class DxlHandle {

public:


private:
    ModelSpec spec;
    int id;
    float protocol;
    std::string name;

    double posRef;

    ControlTable controlTable;

};

}

#endif
