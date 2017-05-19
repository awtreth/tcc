#include <dxl_handle.h>

void dynamixel::DxlHandle::setPosition(double pos){
    set(GOAL_POSITION_ITEM_NAME,spec.radianToValue(pos+posRef));
}

void dynamixel::DxlHandle::setPositionVelocity(double pos, double vel){
    setPosition(pos);
    set(MOVING_SPEED_ITEM_NAME,spec.velocityToValue(vel));
}

void dynamixel::DxlHandle::set(const char *paramName, int value){
    auto ctItem = spec.getControlTableItem(paramName);
    CommandUnit unit(id,uint16_t(ctItem.address));
    ctItem.setWriteCommandUnit(unit,value);
    writeCommandUnits.push_back(unit);
}

void dynamixel::DxlHandle::request(uint16_t addr, uint16_t length){
    readCommandUnits.push_back(CommandUnit(id,addr,length));
}

void dynamixel::DxlHandle::request(const char *paramName){
    auto ctItem = spec.getControlTableItem(paramName);
    request(uint16_t(ctItem.address),uint16_t(ctItem.length));
}

void dynamixel::DxlHandle::requestPosition()
{
    request(PRESENT_POSITION_ITEM_NAME);
}

void dynamixel::DxlHandle::requestVelocity()
{
    request(PRESENT_SPEED_ITEM_NAME);
}

double dynamixel::DxlHandle::getPosition(){
    auto posValue = controlTable.getParam(PRESENT_POSITION_ITEM_NAME);
    return spec.valueToRadian(posValue)-posRef;
}

double dynamixel::DxlHandle::getVelocity() {
    auto velValue = controlTable.getParam(PRESENT_SPEED_ITEM_NAME);
    return spec.valueToVelocity(velValue);
}

void dynamixel::DxlHandle::setReference(const double ref){posRef = ref;}

double dynamixel::DxlHandle::getReference() const{return posRef;}

dynamixel::ModelSpec dynamixel::DxlHandle::getModelSpec() const{return spec;}

uint8_t dynamixel::DxlHandle::getId() const{return id;}

float dynamixel::DxlHandle::getProtocol() const{return protocol;}

std::string dynamixel::DxlHandle::getName() const{return name;}

std::vector<dynamixel::CommandUnit> dynamixel::DxlHandle::getWriteCommandUnits(bool clear)
{
    auto ans =  joinCommandUnits(writeCommandUnits);
    if(clear)
        writeCommandUnits.clear();

    return ans;
}

std::vector<dynamixel::CommandUnit> dynamixel::DxlHandle::getReadCommandUnits(bool clear)
{
    auto ret = joinCommandUnits(readCommandUnits);
    if(clear)
        readCommandUnits.clear();
    return ret;
}

void dynamixel::DxlHandle::clearReadCommandUnits()
{
    readCommandUnits.clear();
}

void dynamixel::DxlHandle::clearWriteCommandUnits()
{
    writeCommandUnits.clear();
}

dynamixel::DxlHandle::DxlHandle(uint8_t _id, float _protocol, dynamixel::ControlTable initControlTable){
    id = _id;
//    handleGroup = mainHandleGroup;
    protocol = _protocol;
    controlTable = initControlTable;
    spec = controlTable.getModelSpec();
    name = spec.getNames()[0] + "_" + std::to_string(id);
}

bool dynamixel::DxlHandle::joinCommandUnits(dynamixel::CommandUnit &first, dynamixel::CommandUnit &second){
    CommandUnit& before = first;
    CommandUnit& after = second;

    if(second.address < first.address){
        before = second;
        after = first;
    }

    if(after.address == before.address+before.getLength()){
        if(before.getData()==NULL && after.getData()==NULL){
            CommandUnit output(id,before.address,before.getLength()+after.getLength());
            before = output;
            after = output;
            return true;
        }else if(before.getData()!=NULL && after.getData()!=NULL){
            before.appendData(after.getData(),after.getLength());
            after = before;
            return true;
        }
    }else if(before.address == after.address && before.getLength() ==after.getLength()){
        first = second;
        return true;
    }

    return false;
}

std::vector<dynamixel::CommandUnit> dynamixel::DxlHandle::joinCommandUnits(std::list<dynamixel::CommandUnit> list){
    std::vector<dynamixel::CommandUnit> vec;

    while(list.size()>0){
        auto unit = list.front();
        list.pop_front();
        for(auto it = list.begin(); it != list.end(); it++){
            if(joinCommandUnits(unit,*it)){
                list.erase(it);
                it = list.begin();
            }
        }
        vec.push_back(unit);
    }

    return vec;
}
