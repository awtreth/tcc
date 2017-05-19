#include <dxl_control_table.h>

dynamixel::ControlTable::ControlTable(){}

dynamixel::ControlTable::ControlTable(dynamixel::ModelSpec modelSpec):ControlTable(modelSpec.getControlTableSize()){
    spec = modelSpec;
}

dynamixel::ControlTable::ControlTable(int size){
    controlTableVec.resize(static_cast <unsigned int>(size));
    controlTable = controlTableVec.data();
}

void dynamixel::ControlTable::setParam(const char *paramName, uint8_t *data){
    auto item = spec.getControlTableItem(paramName);
    set(item.address,item.length,data);
}

void dynamixel::ControlTable::getParam(const char *paramName, uint8_t *data){
    auto item = spec.getControlTableItem(paramName);
    get(item.address,item.length,data);
}

int dynamixel::ControlTable::getParam(const char *paramName){
    auto item = spec.getControlTableItem(paramName);

    int data = 0;

    get(item.address,item.length,&data);

    if(item.length == 1)
        data = int(reinterpret_cast<uint8_t*>(&data)[0]);
    else if (item.length == 2)
        data = int(reinterpret_cast<uint16_t*>(&data)[0]);

    return data;
}

dynamixel::ModelSpec dynamixel::ControlTable::getModelSpec() const {return spec;}

uint8_t *dynamixel::ControlTable::getPtr(int address){
    return &controlTable[address];
}

size_t dynamixel::ControlTable::size(){
    return controlTableVec.size();
}
