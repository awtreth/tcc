#ifndef DXL_COMMAND_H
#define DXL_COMMAND_H

#include <dynamixel_sdk.h>

#include <vector>
#include <iostream>
#include <stdarg.h>
#include <stdexcept>
#include <algorithm>

namespace dxl_interface {

class CommandUnit {

public:
    uint16_t address = 0;
    uint8_t id = 0;

    CommandUnit();

    template<typename T = uint8_t>
    CommandUnit(uint8_t _id, uint16_t _address, uint16_t _length = 0, T* data = NULL){
        id = _id;
        address = _address;
        length = _length;
        setData(data, _length);
    }

    template<typename T>
    bool setData(T* data, uint16_t dataLength = 0){

        if(data==NULL || (dataLength==0 && length==0))
            return false;

        if(dataLength>0)
            length = dataLength;

        dataVec.resize(size_t(length));

        std::copy(reinterpret_cast<uint8_t*>(data),reinterpret_cast<uint8_t*>(data)+length,&dataVec[0]);

        return true;
    }

    template<typename T>
    bool appendData(T* data, uint16_t appendLength){
        if(data==NULL || appendLength == 0)
            return false;

        auto oldSize = length;
        length += appendLength;
        dataVec.resize(length);

        std::copy(data,data+appendLength,dataVec.begin()+oldSize);

        return true;
    }

    uint8_t* getData();

    void clearData();

    uint16_t getLength() const;

private:
    uint16_t length;
    std::vector<uint8_t> dataVec;

};


class AbsCommand {

public:
    int getInstruction() const;
    float getProtocol() const;

    void addCommandUnit(CommandUnit unit);

    template<typename T>
    bool addCommandUnit(uint8_t id, T* data = NULL){

        if(getSize()>0){
            CommandUnit unit(id,getFirst().address,getFirst().getLength(),data);
            commandUnits.push_back(unit);
            return true;
        }
        return false;
    }

    CommandUnit getFirst();

    std::vector<CommandUnit> getCommandUnits() const;

    size_t getSize(){
        return commandUnits.size();
    }

protected:

    virtual ~AbsCommand();

    std::vector<CommandUnit> commandUnits;

    int instruction;
    float protocol;

    virtual void init(int inst, int protc);

    virtual bool areInstructionProtocolValid(int _instruction, int _protocol) = 0;
    virtual void checkInstructionUnitCompatibility(CommandUnit testUnit) = 0;

    //    template<typename T1, typename T2>
    //    bool compareSizes(std::vector<T1> vec1, std::vector<T2> vec2){return vec1.size() == vec2.size();}

    //    template<typename T1, typename T2, typename ...TN>
    //    bool compareSizes(std::vector<T1> vec1, std::vector<T2> vec2, std::vector<TN> ...vecs){
    //        return compareSizes(vec1,vec2) && compareSizes(vec2,vecs...);
    //    }

    //    virtual bool areInstructionUnitCompatible(CommandUnit testUnit) = 0;
    bool areInstructionUnitCompatibleBase(CommandUnit tested, int syncInstruction);

};


class ReadCommand : public AbsCommand{

public:
    ReadCommand(int _instruction, float _protocol);
    ReadCommand(int _instruction, float _protocol, CommandUnit firstUnit);

protected:

    virtual bool areInstructionProtocolValid(int inst, int protoc);

    virtual void checkInstructionUnitCompatibility(CommandUnit testUnit);

};

class WriteCommand : public AbsCommand{

public:

    WriteCommand(const int _instruction, const float _protocol);
    WriteCommand(const int _instruction, const float _protocol, CommandUnit firstUnit);

protected:

    virtual bool areInstructionProtocolValid(int inst, int protoc);

    virtual void checkInstructionUnitCompatibility(CommandUnit testUnit);
};

}
#endif
