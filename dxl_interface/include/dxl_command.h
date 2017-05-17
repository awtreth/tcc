#ifndef DXL_COMMAND_H
#define DXL_COMMAND_H

#include <dynamixel_sdk.h>

#include <vector>
#include <iostream>
#include <stdarg.h>
#include <stdexcept>

namespace dynamixel {

class AbsDxlCommand {

protected:

    int instruction;
    float protocol;
    std::vector<int> ids;
    std::vector<int> addresses;
    std::vector<int> lengths;

    virtual void init(int inst, int protc){
        if(isInstructionProtocolValid(inst,protc)){
            instruction = inst;
            protocol = protc;
        }else
            throw std::invalid_argument("DxlCommand: instruction/protocol not compatible");
    }

    template<typename T1, typename T2>
    bool compareSizes(std::vector<T1> vec1, std::vector<T2> vec2){return vec1.size() == vec2.size();}

    template<typename T1, typename T2, typename ...TN>
    bool compareSizes(std::vector<T1> vec1, std::vector<T2> vec2, std::vector<TN> ...vecs){
        return compareSizes(vec1,vec2) && compareSizes(vec2,vecs...);
    }

    virtual bool isInstructionProtocolValid(int _instruction, int _protocol) = 0;
};



class DxlReadCommand : public AbsDxlCommand{

public:

    DxlReadCommand(int _instruction, float _protocol){
        init(_instruction,int(_protocol));
    }

    DxlReadCommand(int _instruction, float _protocol, std::vector<int> _ids,
                   std::vector<int> _addresses, std::vector<int> _lengths)
        :DxlReadCommand(_instruction,_protocol){

        addCommand(_ids,_addresses,_lengths);
    }

    void addCommand(int id, int address, int length){
        ids.push_back(id);
        addresses.push_back(address);
        lengths.push_back(length);
    }

    void addCommand(std::vector<int> _ids, std::vector<int> _addresses, std::vector<int> _lengths){

        if(compareSizes(_ids,_addresses,_lengths)==false)
            throw std::invalid_argument("the size of input vectors don't match");

        ids.insert(ids.end(),_ids.begin(),_ids.end());
        addresses.insert(addresses.end(),_addresses.begin(),_addresses.end());
        lengths.insert(lengths.end(),_lengths.begin(),_lengths.end());
    }

protected:

    virtual bool isInstructionProtocolValid(int inst, int protoc){
        if((inst == INST_BULK_READ || inst == INST_READ || inst == INST_SYNC_READ)
                && (protoc == 1 || protoc == 2))
            return !(inst == INST_SYNC_READ && protoc == 1);//Protocol 1.0 does not support SYNC READ
        else
            return false;
    }
};

class DxlWriteCommand : public AbsDxlCommand{

public:

    DxlWriteCommand(const int _instruction, const float _protocol){
        init(_instruction,int(_protocol));
    }

    DxlWriteCommand(int _instruction, float _protocol, std::vector<int> _ids,
                    std::vector<int> _addresses, std::vector<int> _lengths,
                    std::vector<uint8_t*> _values)
        :DxlWriteCommand(_instruction,_protocol){

        addCommand(_ids,_addresses,_lengths,_values);
    }

protected:

    std::vector<uint8_t*> values;

    void addCommand(int id, int address, int length, uint8_t* value){
        ids.push_back(id);
        addresses.push_back(address);
        lengths.push_back(length);
        values.push_back(value);
    }

    void addCommand(std::vector<int> _ids, std::vector<int> _addresses, std::vector<int> _lengths, std::vector<uint8_t*> _values){

        if(compareSizes(_ids,_addresses,_lengths,_values)==false)
            throw std::invalid_argument("DxlWriteCommand: the size of input vectors don't match");

        ids.insert(ids.end(),_ids.begin(),_ids.end());
        addresses.insert(addresses.end(),_addresses.begin(),_addresses.end());
        lengths.insert(lengths.end(),_lengths.begin(),_lengths.end());
        values.insert(values.end(),_values.begin(),_values.end());
    }

    virtual bool isInstructionProtocolValid(int inst, int protoc){
        if((inst == INST_WRITE || inst == INST_BULK_WRITE || inst == INST_SYNC_WRITE || inst == INST_REG_WRITE)
                && (protoc == 1 || protoc == 2))
            return !(inst == INST_BULK_WRITE && protoc == 1);//Protocol 1.0 does not support BULK WRITE
        else
            return false;
    }

};

}
#endif
