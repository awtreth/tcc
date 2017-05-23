#include <dxl_control_table.h>

dxl_interface::ControlTable::ControlTable(){}

dxl_interface::ControlTable::ControlTable(dxl_interface::ModelSpec modelSpec):ControlTable(modelSpec.getControlTableSize()){
    spec = modelSpec;
}

dxl_interface::ControlTable::ControlTable(int size){
    controlTableVec.resize(static_cast <unsigned int>(size));
    controlTable = controlTableVec.data();
}

void dxl_interface::ControlTable::setParam(const char *paramName, uint8_t *data){
    auto item = spec.getControlTableItem(paramName);
    set(item.address,item.length,data);
}

void dxl_interface::ControlTable::getParam(const char *paramName, uint8_t *data){
    auto item = spec.getControlTableItem(paramName);
    get(item.address,item.length,data);
}

int dxl_interface::ControlTable::getParam(const char *paramName){
    auto item = spec.getControlTableItem(paramName);

    int data = 0;

    get(item.address,item.length,&data);

    if(item.length == 1)
        data = int(reinterpret_cast<uint8_t*>(&data)[0]);
    else if (item.length == 2)
        data = int(reinterpret_cast<uint16_t*>(&data)[0]);

    return data;
}

dxl_interface::ModelSpec dxl_interface::ControlTable::getModelSpec() const {return spec;}

uint8_t *dxl_interface::ControlTable::getPtr(int address){
    return &controlTable[address];
}

size_t dxl_interface::ControlTable::size(){
    return controlTableVec.size();
}
