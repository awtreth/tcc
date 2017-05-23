#ifndef DXLMEMMAP_H
#define DXLMEMMAP_H

#include <vector>
#include <string>
#include <algorithm>
#include <dxl_interface/dxl_model_spec.h>

namespace dxl_interface {

/// Classe que representa/simula a tabela de controle (EEPROM e RAM) de motores Dynamixel.
/// Veja mais detalhes sobre a tabela para, por exemplo, os motores MX-106 no link <a href="http://support.robotis.com/en/product/dynamixel/mx_series/mx-106.htm">link</a>.
/// \todo aprimorar gerenciamento de memória. As funções consideram que o input está correto. Implementar exceções
class ControlTable {
    private:

    std::vector<uint8_t> controlTableVec;
    uint8_t* controlTable;
    ModelSpec spec;

    public:

    ControlTable();

    ControlTable(ModelSpec modelSpec);

    ControlTable(int size);

    void setParam(const char* paramName, uint8_t* data);

    void getParam(const char* paramName, uint8_t* data);

    int getParam(const char* paramName);

    ModelSpec getModelSpec() const;

    /// Altera valores da tabela de controle C Style.
    /// Esta versão foi criada por motivos históricos, semelhante ao código atual da EDROM. Possui um estilo mais C.
    /// \param startAddress Endereço da tabela de controle do motor inicial para escrita
    /// \param length Número de bytes a serem alterados a partir do startAddress
    /// \param inValuesPtr Ponteiro para os dados de entrada. A função considera que esteja devidamente alocado e de acordo com o tamanho (length passado).O usuário deve tomar cuidado com o gerenciamento de memória
    /// \see set()
    template<typename T>
    void set(int startAddress, int length, T* inValuesPtr){
        std::copy(reinterpret_cast<uint8_t*>(inValuesPtr), reinterpret_cast<uint8_t*>(inValuesPtr)+length,
                  &controlTable[startAddress]);
    }

    /// Lê valores da tabela de controle C Style.
    /// Esta versão foi criada por motivos históricos, semelhante ao código atual da EDROM. Possui um estilo mais C.
    /// \param startAddress Endereço da tabela de controle do motor inicial para leitura
    /// \param length Número de bytes a serem lidos a partir de startAddress
    /// \param outValuesPtrPonteiro para os dados de saída. A função considera que esteja devidamente alocado e de acordo com o tamanho (length passado).O usuário deve tomar cuidado com o gerenciamento de memória
    /// \see get()
    template<typename T>
    void get(int startAddress, int length, T* outValuesPtr){
        std::copy(&controlTable[startAddress],&controlTable[startAddress+length], reinterpret_cast<uint8_t*>(outValuesPtr));
    }

    uint8_t* getPtr(int address);

    size_t size();
};

}
#endif // DXLMEMMAP_H
