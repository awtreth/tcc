#ifndef DXLMEMMAP_H
#define DXLMEMMAP_H

#include <vector>
#include <string>
#include "dxl_constants_protocol_1.h"
#include <algorithm>

namespace dynamixel {

/// Classe que representa/simula a tabela de controle (EEPROM e RAM) de motores Dynamixel.
/// Veja mais detalhes sobre a tabela para, por exemplo, os motores MX-106 no link <a href="http://support.robotis.com/en/product/dynamixel/mx_series/mx-106.htm">link</a>.
/// \todo aprimorar gerenciamento de memória. As funções consideram que o input está correto. Implementar exceções
class ControlTable {
    private:

    std::vector<uint8_t> controlTableVec;
    uint8_t* controlTable;

    public:

    ControlTable(){
        controlTableVec.resize(CONTROL_TABLE_SIZE_1);
        controlTable = controlTableVec.data();
    }

    ControlTable(unsigned int size){
        controlTableVec.resize(size);
        controlTable = controlTableVec.data();
    }

    /// Altera valores da tabela de controle C Style.
    /// Esta versão foi criada por motivos históricos, semelhante ao código atual da EDROM. Possui um estilo mais C.
    /// \param startAddress Endereço da tabela de controle do motor inicial para escrita
    /// \param length Número de bytes a serem alterados a partir do startAddress
    /// \param inValuesPtr Ponteiro para os dados de entrada. A função considera que esteja devidamente alocado e de acordo com o tamanho (length passado).O usuário deve tomar cuidado com o gerenciamento de memória
    /// \see set()
    void set(int startAddress, int length, uint8_t* inValuesPtr){
        std::copy(inValuesPtr, inValuesPtr+length, &controlTable[startAddress]);
    }

    /// Lê valores da tabela de controle C Style.
    /// Esta versão foi criada por motivos históricos, semelhante ao código atual da EDROM. Possui um estilo mais C.
    /// \param startAddress Endereço da tabela de controle do motor inicial para leitura
    /// \param length Número de bytes a serem lidos a partir de startAddress
    /// \param outValuesPtrPonteiro para os dados de saída. A função considera que esteja devidamente alocado e de acordo com o tamanho (length passado).O usuário deve tomar cuidado com o gerenciamento de memória
    /// \see get()
    void get(int startAddress, int length, char* outValuesPtr){
        std::copy(&controlTable[startAddress],&controlTable[startAddress+length], outValuesPtr);
    }

};

}
#endif // DXLMEMMAP_H
