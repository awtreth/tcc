#ifndef DXL_HANDLE_H
#define DXL_HANDLE_H

#include <dxl_model_spec.h>
#include <dxl_control_table.h>
#include <dxl_command.h>
#include <string>
#include <list>

namespace dynamixel {

class DxlHandle {

public:


    template<typename T = uint8_t>
    void set(uint16_t addr, uint16_t length, T* data){
        writeCommandUnits.push_back(CommandUnit(id,addr,length,data));
    }
    void set(const char* paramName, int value);

    void request(uint16_t addr, uint16_t length);
    void request(const char* paramName);

    void setPosition(double pos);
    void setPositionVelocity(double pos, double vel);

    void requestPosition();
    void requestVelocity();

//    bool requestJointState();

    double getPosition();
    double getVelocity();

    void setReference(const double ref);
    double getReference() const;

    ModelSpec getModelSpec() const;
    uint8_t getId() const;
    float getProtocol() const;
    std::string getName() const;

    std::vector<CommandUnit> getReadCommandUnits(bool clear = true);
    std::vector<CommandUnit> getWriteCommandUnits(bool clear = true);

    void clearReadCommandUnits();
    void clearWriteCommandUnits();

private:

    friend class DxlChannel;

    std::list<CommandUnit> readCommandUnits;
    std::list<CommandUnit> writeCommandUnits;

//    DxlHandle(uint8_t _id, float _protocol, ControlTable initControlTable, DxlHandleGroup* mainHandleGroup);
    DxlHandle(uint8_t _id, float _protocol, ControlTable initControlTable);

    ModelSpec spec;
    uint8_t id;
    float protocol;
    std::string name;

    double posRef = 0;

    ControlTable controlTable;

//    DxlHandleGroup* handleGroup;

    bool joinCommandUnits(CommandUnit& first, CommandUnit& second);

    std::vector<CommandUnit> joinCommandUnits(std::list<CommandUnit> list);

};

}

#endif
