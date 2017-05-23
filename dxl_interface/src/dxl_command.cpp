#include <dxl_command.h>

dxl_interface::CommandUnit::CommandUnit(){}

uint8_t *dxl_interface::CommandUnit::getData(){
    if(dataVec.size()>0)
        return dataVec.data();
    else
        return NULL;
}

void dxl_interface::CommandUnit::clearData(){
    dataVec.clear();
}

uint16_t dxl_interface::CommandUnit::getLength() const {
    return length;
}

int dxl_interface::AbsCommand::getInstruction() const{return instruction;}

float dxl_interface::AbsCommand::getProtocol() const{return protocol;}

void dxl_interface::AbsCommand::addCommandUnit(dxl_interface::CommandUnit unit){
    checkInstructionUnitCompatibility(unit);
    commandUnits.push_back(unit);
}

dxl_interface::CommandUnit dxl_interface::AbsCommand::getFirst(){
    return commandUnits.front();
}

std::vector<dxl_interface::CommandUnit> dxl_interface::AbsCommand::getCommandUnits() const{
    return commandUnits;
}

dxl_interface::AbsCommand::~AbsCommand(){}

void dxl_interface::AbsCommand::init(int inst, int protc){
    if(areInstructionProtocolValid(inst,protc)){
        instruction = inst;
        protocol = protc;
    }else
        throw std::invalid_argument("Instruction/protocol not compatible");

}

bool dxl_interface::AbsCommand::areInstructionUnitCompatibleBase(dxl_interface::CommandUnit tested, int syncInstruction){
    if(getSize()==0 || instruction != syncInstruction)
        return true;
    else
        return commandUnits[0].address == tested.address && commandUnits[0].getLength() == tested.getLength();
}

dxl_interface::ReadCommand::ReadCommand(int _instruction, float _protocol)
{
    init(_instruction,int(_protocol));
}

dxl_interface::ReadCommand::ReadCommand(int _instruction, float _protocol, dxl_interface::CommandUnit firstUnit){
    init(_instruction,int(_protocol));
    commandUnits.push_back(firstUnit);
}

bool dxl_interface::ReadCommand::areInstructionProtocolValid(int inst, int protoc){
    if((inst == INST_BULK_READ || inst == INST_READ || inst == INST_SYNC_READ)
            && (protoc == 1 || protoc == 2))
        return !(inst == INST_SYNC_READ && protoc == 1);//Protocol 1.0 does not support SYNC READ
    else
        return false;
}

void dxl_interface::ReadCommand::checkInstructionUnitCompatibility(dxl_interface::CommandUnit testUnit){
    if(!areInstructionUnitCompatibleBase(testUnit,INST_SYNC_READ))
        throw std::invalid_argument("INST_SYNC_READ require same address and length for all units (based on first unit)");
}

dxl_interface::WriteCommand::WriteCommand(const int _instruction, const float _protocol)
{
    init(_instruction,int(_protocol));
}

dxl_interface::WriteCommand::WriteCommand(const int _instruction, const float _protocol, dxl_interface::CommandUnit firstUnit){
    init(_instruction,int(_protocol));
    commandUnits.push_back(firstUnit);
}

bool dxl_interface::WriteCommand::areInstructionProtocolValid(int inst, int protoc){
    if((inst == INST_WRITE || inst == INST_BULK_WRITE || inst == INST_SYNC_WRITE || inst == INST_REG_WRITE)
            && (protoc == 1 || protoc == 2))
        return !(inst == INST_BULK_WRITE && protoc == 1);//Protocol 1.0 does not support BULK WRITE
    else
        return false;
}

void dxl_interface::WriteCommand::checkInstructionUnitCompatibility(dxl_interface::CommandUnit testUnit){
    if(!areInstructionUnitCompatibleBase(testUnit,INST_SYNC_WRITE))
        throw std::invalid_argument("INST_SYNC_WRITE require same address and length for all units (based on first unit)");
}
