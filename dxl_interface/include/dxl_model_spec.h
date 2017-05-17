#ifndef DXL_SPEC_H
#define DXL_SPEC_H

#include <string>
#include <vector>

#define DEFAULT_MODEL_SPEC_RELATIVE_FOLDER  "model_specs/"
#define DEFAULT_MODEL_SPEC_FILE_EXTENSION   ".dxl"

namespace dynamixel {

class ModelSpec {

private:
    std::vector<std::string> names;
    std::vector<int> numbers;

    double valueToPositionRatio;
    double valueToVelocityRatio;

    int zeroPositionValue;
    int zeroVelocityValue;//for wheel mode and read operations

    static std::vector<std::string> listFiles(const char* folder);

public:

    ModelSpec();

    ModelSpec(const char* fileName);

    ModelSpec(const std::vector<std::string> _names, const std::vector<int> _numbers,
              const double valueToPosRatio, const double valueToVelRatio,
              const int zeroPosValue, const int zeroVelValue);

    static ModelSpec getByNumber(int modelNumber, const char* folder = DEFAULT_MODEL_SPEC_RELATIVE_FOLDER);

    static ModelSpec getByName(const char* modelName, const char* folder = DEFAULT_MODEL_SPEC_RELATIVE_FOLDER);

    /**
         * @brief Converte posição em unidade de motor para radianos
         *
         * @param pos Posição em unidade de motor
         * @return double Posição em radianos
         */
    double valueToRadian(int posValue, bool wheelModeOrRead = true);

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
};

}//namespace bracket

#endif
