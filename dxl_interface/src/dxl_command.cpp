#include <dxl_command.h>

dynamixel::CommandUnit::CommandUnit(){}

uint8_t *dynamixel::CommandUnit::getData(){
    if(dataVec.size()>0)
        return dataVec.data();
    else
        return NULL;
}

void dynamixel::CommandUnit::clearData(){
    dataVec.clear();
}

uint16_t dynamixel::CommandUnit::getLength() const {
    return length;
}

int dynamixel::AbsCommand::getInstruction() const{return instruction;}

float dynamixel::AbsCommand::getProtocol() const{return protocol;}

void dynamixel::AbsCommand::addCommandUnit(dynamixel::CommandUnit unit){
    checkInstructionUnitCompatibility(unit);
    commandUnits.push_back(unit);
}

dynamixel::CommandUnit dynamixel::AbsCommand::getFirst(){
    return commandUnits.front();
}

std::vector<dynamixel::CommandUnit> dynamixel::AbsCommand::getCommandUnits() const{
    return commandUnits;
}

dynamixel::AbsCommand::~AbsCommand(){}

void dynamixel::AbsCommand::init(int inst, int protc){
    if(areInstructionProtocolValid(inst,protc)){
        instruction = inst;
        protocol = protc;
    }else
        throw std::invalid_argument("Instruction/protocol not compatible");

}

bool dynamixel::AbsCommand::areInstructionUnitCompatibleBase(dynamixel::CommandUnit tested, int syncInstruction){
    if(getSize()==0 || instruction != syncInstruction)
        return true;
    else
        return commandUnits[0].address == tested.address && commandUnits[0].getLength() == tested.getLength();
}

dynamixel::ReadCommand::ReadCommand(int _instruction, float _protocol)
{
    init(_instruction,int(_protocol));
}

dynamixel::ReadCommand::ReadCommand(int _instruction, float _protocol, dynamixel::CommandUnit firstUnit){
    init(_instruction,int(_protocol));
    commandUnits.push_back(firstUnit);
}

bool dynamixel::ReadCommand::areInstructionProtocolValid(int inst, int protoc){
    if((inst == INST_BULK_READ || inst == INST_READ || inst == INST_SYNC_READ)
            && (protoc == 1 || protoc == 2))
        return !(inst == INST_SYNC_READ && protoc == 1);//Protocol 1.0 does not support SYNC READ
    else
        return false;
}

void dynamixel::ReadCommand::checkInstructionUnitCompatibility(dynamixel::CommandUnit testUnit){
    if(!areInstructionUnitCompatibleBase(testUnit,INST_SYNC_READ))
        throw std::invalid_argument("INST_SYNC_READ require same address and length for all units (based on first unit)");
}

dynamixel::WriteCommand::WriteCommand(const int _instruction, const float _protocol)
{
    init(_instruction,int(_protocol));
}

dynamixel::WriteCommand::WriteCommand(const int _instruction, const float _protocol, dynamixel::CommandUnit firstUnit){
    init(_instruction,int(_protocol));
    commandUnits.push_back(firstUnit);
}

bool dynamixel::WriteCommand::areInstructionProtocolValid(int inst, int protoc){
    if((inst == INST_WRITE || inst == INST_BULK_WRITE || inst == INST_SYNC_WRITE || inst == INST_REG_WRITE)
            && (protoc == 1 || protoc == 2))
        return !(inst == INST_BULK_WRITE && protoc == 1);//Protocol 1.0 does not support BULK WRITE
    else
        return false;
}

void dynamixel::WriteCommand::checkInstructionUnitCompatibility(dynamixel::CommandUnit testUnit){
    if(!areInstructionUnitCompatibleBase(testUnit,INST_SYNC_WRITE))
        throw std::invalid_argument("INST_SYNC_WRITE require same address and length for all units (based on first unit)");
}
