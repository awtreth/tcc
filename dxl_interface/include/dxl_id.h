#ifndef DXL_ID_H
#define DXL_ID_H

#include <string>

namespace dynamixel {


/**
 * @brief Information needed to communicate with a Dynamixel Motor through the DxlChannel
 *
 * The model is necessary, because the control table may differ.
 *
 */
struct DxlId {

    std::string name;
    int id = -1;
    int protocol = -1;
    int modelNumber = -1;
    std::string modelName;

    DxlId(){}

    DxlId(int _id):id(_id){}

};

}

#endif
