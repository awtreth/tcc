#ifndef DXL_SPEC_H
#define DXL_SPEC_H

#include <string>
#include <cmath>

namespace dynamixel {

class ModelSpec {
private:
    std::string name;
    int number;

    double valueToPositionRatio;
    double valueToVelocityRatio;

    int positionValueRange;
    int maxJointModeVelocityValue = 1023;

    int zeroPositionValue;
    int zeroVelocityValue = 1024;//for wheel mode and read operations

public:

    ModelSpec(const char* fileName);

    ModelSpec(const std::string _name, const int _number, const double valueToPosRatio, const double valueToVelRatio,
              const int posValueRange, const int maxVelocity, const int zeroPosValue, const int zeroVelValue){
        name = _name;
        number = _number;
        valueToPositionRatio = valueToPosRatio;
        valueToVelocityRatio = valueToVelRatio;
        positionValueRange = posValueRange;
        maxJointModeVelocityValue = maxVelocity;
        zeroPositionValue = zeroPosValue;
        zeroVelocityValue = zeroVelValue;
    }

    /**
         * @brief Converte posição em unidade de motor para radianos
         *
         * @param pos Posição em unidade de motor
         * @return double Posição em radianos
         */
    double valueToRadian(int posValue, bool wheelModeOrRead = true){
        if(wheelModeOrRead)
            return (posValue-zeroPositionValue)*valueToPositionRatio;
        else
            return posValue*valueToPositionRatio;
    }

    /**
         * @brief Converte velocidade em unidade de motor para radianos por segundo
         *
         * @param vel Velocidade em unidade de motor
         * @return double Velocidade em radianos por segundo
         */
    double valueToVelocity(int velValue){
        return (velValue-zeroVelocityValue)*valueToVelocityRatio;
    }


    int radianToValue(double pos){
        return int(pos/valueToPositionRatio+zeroPositionValue);
    }

    int velocityToValue(double vel, bool wheelMode = false){
        if(wheelMode)
            return int(vel/valueToVelocityRatio+zeroVelocityValue);
        else
            return int(vel/valueToVelocityRatio);
    }

};



}

#endif
