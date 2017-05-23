#include <dxl_handle.h>

void dxl_interface::DxlHandle::setPosition(double pos){
    set(GOAL_POSITION_ITEM_NAME,spec.radianToValue(pos+posRef));
}

void dxl_interface::DxlHandle::setPositionVelocity(double pos, double vel){
    setPosition(pos);
    set(MOVING_SPEED_ITEM_NAME,spec.velocityToValue(vel));
}

void dxl_interface::DxlHandle::set(const char *paramName, int value){
    auto ctItem = spec.getControlTableItem(paramName);
    CommandUnit unit(id,uint16_t(ctItem.address));
    ctItem.setWriteCommandUnit(unit,value);
    writeCommandUnits.push_back(unit);
}

void dxl_interface::DxlHandle::request(uint16_t addr, uint16_t length){
    readCommandUnits.push_back(CommandUnit(id,addr,length));
}

void dxl_interface::DxlHandle::request(const char *paramName){
    auto ctItem = spec.getControlTableItem(paramName);
    request(uint16_t(ctItem.address),uint16_t(ctItem.length));
}

void dxl_interface::DxlHandle::requestPosition()
{
    request(PRESENT_POSITION_ITEM_NAME);
}

void dxl_interface::DxlHandle::requestVelocity()
{
    request(PRESENT_SPEED_ITEM_NAME);
}

double dxl_interface::DxlHandle::getPosition(){
    auto posValue = controlTable.getParam(PRESENT_POSITION_ITEM_NAME);
    return spec.valueToRadian(posValue)-posRef;
}

double dxl_interface::DxlHandle::getVelocity() {
    auto velValue = controlTable.getParam(PRESENT_SPEED_ITEM_NAME);
    return spec.valueToVelocity(velValue);
}

void dxl_interface::DxlHandle::setReference(const double ref){posRef = ref;}

double dxl_interface::DxlHandle::getReference() const{return posRef;}

dxl_interface::ModelSpec dxl_interface::DxlHandle::getModelSpec() const{return spec;}

uint8_t dxl_interface::DxlHandle::getId() const{return id;}

float dxl_interface::DxlHandle::getProtocol() const{return protocol;}

std::string dxl_interface::DxlHandle::getName() const{return name;}

std::vector<dxl_interface::CommandUnit> dxl_interface::DxlHandle::getWriteCommandUnits(bool clear)
{
    auto ans =  joinCommandUnits(writeCommandUnits);
    if(clear)
        writeCommandUnits.clear();

    return ans;
}

std::vector<dxl_interface::CommandUnit> dxl_interface::DxlHandle::getReadCommandUnits(bool clear)
{
    auto ret = joinCommandUnits(readCommandUnits);
    if(clear)
        readCommandUnits.clear();
    return ret;
}

void dxl_interface::DxlHandle::clearReadCommandUnits()
{
    readCommandUnits.clear();
}

void dxl_interface::DxlHandle::clearWriteCommandUnits()
{
    writeCommandUnits.clear();
}

dxl_interface::DxlHandle::DxlHandle(uint8_t _id, float _protocol, dxl_interface::ControlTable initControlTable){
    id = _id;
//    handleGroup = mainHandleGroup;
    protocol = _protocol;
    controlTable = initControlTable;
    spec = controlTable.getModelSpec();
    name = spec.getNames()[0] + "_" + std::to_string(id);
}

bool dxl_interface::DxlHandle::joinCommandUnits(dxl_interface::CommandUnit &first, dxl_interface::CommandUnit &second){
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

std::vector<dxl_interface::CommandUnit> dxl_interface::DxlHandle::joinCommandUnits(std::list<dxl_interface::CommandUnit> list){
    std::vector<dxl_interface::CommandUnit> vec;

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
