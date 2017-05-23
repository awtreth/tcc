#ifndef DXL_SPEC_H
#define DXL_SPEC_H

#include <string>
#include <vector>
#include <map>
#include <dxl_interface/dxl_command.h>

#define DEFAULT_MODEL_SPEC_FOLDER  "model_specs/"
#define DEFAULT_MODEL_SPEC_FILE_EXTENSION   ".dxl"

#define MODEL_ITEM_NAME             "model"
#define ID_ITEM_NAME                "id"
#define GOAL_POSITION_ITEM_NAME     "goal_position"
#define MOVING_SPEED_ITEM_NAME      "moving_speed"
#define PRESENT_POSITION_ITEM_NAME  "present_position"
#define PRESENT_SPEED_ITEM_NAME     "present_speed"

#define ADDRESS_CT_ITEM_NAME    "address"
#define LENGTH_CT_ITEM_NAME     "length"
#define ACCESS_CT_ITEM_NAME     "access"

namespace dxl_interface {

struct ControlTableItem{
    std::string name = "";
    int address = -1;
    int length = -1;
    bool isWritable = false;

    bool setWriteCommandUnit(CommandUnit& unit, int value){

        if(length == 1){
            auto data = uint8_t(value);
            unit.setData(&data,1);
        }else if (length == 2){
            uint16_t data = uint16_t(value);
            unit.setData(&data,2);
        }else if (length == 4)
            unit.setData(&value,4);
        else
            return false;

        return true;
    }
};

//Inspired on https://github.com/ROBOTIS-GIT/ROBOTIS-Framework
class ModelSpec {

private:
    std::map<std::string,ControlTableItem> ctItems;

    std::vector<std::string> names;
    std::vector<int> numbers;

    double valueToPositionRatio = 0;
    double valueToVelocityRatio = 0;

    int controlTableSize = 0;
    float protocol = 1.0;

    static std::vector<std::string> listFiles(const char* folder, const char *file_extension);

public:

    ModelSpec();

    ModelSpec(const char* fileName);

    bool isValid();

    static ModelSpec getByNumber(int modelNumber, const char* folder = DEFAULT_MODEL_SPEC_FOLDER, const char* file_extension = DEFAULT_MODEL_SPEC_FILE_EXTENSION);
    static ModelSpec getByName(const char* modelName, const char* folder = DEFAULT_MODEL_SPEC_FOLDER, const char* file_extension = DEFAULT_MODEL_SPEC_FILE_EXTENSION);

    ControlTableItem getControlTableItem(const char* name);

    /**
         * @brief Converte posição em unidade de motor para radianos
         *
         * @param pos Posição em unidade de motor
         * @return double Posição em radianos
         */
    double valueToRadian(int posValue);

    /**
         * @brief Converte velocidade em unidade de motor para radianos por segundo
         *
         * @param vel Velocidade em unidade de motor
         * @return double Velocidade em radianos por segundo
         */
    double valueToVelocity(int velValue);


    int radianToValue(double pos);

    int velocityToValue(double vel, bool wheelMode = false);

    std::vector<std::string> getNames() const;
    std::vector<int> getNumbers() const;

    std::string toString();
    double getValueToPositionRatio() const;
    double getValueToVelocityRatio() const;

    bool hasName(const char* name);
    bool hasNumber(const int number);
    bool hasNameLike(const char* name);
    int getControlTableSize() const;
};

}//namespace bracket


#endif
