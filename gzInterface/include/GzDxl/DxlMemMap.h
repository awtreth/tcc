#ifndef DXLMEMMAP_H
#define DXLMEMMAP_H


#include "DxlMemMapConstants.h"
#include <vector>
#include <string>

/// Classe que representa/simula o mapa da memória EEPROM e RAM dos motores Dynamixel.
/// Classe que representa/simula a tabela de controle de motores Dynamixel.
/// Veja mais detalhes sobre a tabela para, por exemplo, os motores MX-106 no link <a href="http://support.robotis.com/en/product/dynamixel/mx_series/mx-106.htm">link</a>.
/// \todo aprimorar gerenciamento de memória. As funções consideram que o input está correto. Implementar exceções
class DxlMemMap {
	private:
	
#define MAX_CONTROL_TABLE_SIZE 100

	/// Vetor que armazena os valores da tabela
	char controlTable[MAX_CONTROL_TABLE_SIZE];

	/// Vetor auxiliar com nomes dos campos extraídos de arquivo json
	std::vector<std::string> fieldNames;


	public:
	
	/// Construtor padrão. Cria uma tabela com todos os campos igual a zero.
	DxlMemMap();

	/// Carrega o mapa contido no arquivo .dxlmap (JSON File).
	/// \param jsonFileName Caminho do arquivo contendo configuração de mapa de memória de motor. Por padrão, estabeleceu-se a extensão .dxlmap para estes arquivos.
	DxlMemMap(const char* jsonFileName);

	/// Altera valores da tabela de controle C Style.
	/// Esta versão foi criada por motivos históricos, semelhante ao código atual da EDROM. Possui um estilo mais C.
	/// \param startAddress Endereço da tabela de controle do motor inicial para escrita
	/// \param length Número de bytes a serem alterados a partir do startAddress
	/// \param inValuesPtr Ponteiro para os dados de entrada. A função considera que esteja devidamente alocado e de acordo com o tamanho (length passado).O usuário deve tomar cuidado com o gerenciamento de memória
	/// \see set()
	void set(int startAddress, int length, char* inValuesPtr);

	/// Altera valores da tabela de controle C++ Style orientado a byte.
	/// Versão mais simples de ser usada. O tamanho (length) é tamanho dos dados contidos no vector em bytes
	/// \param startAddress Endereço da tabela de controle do motor inicial para escrita
	/// \param values Valores para serem escritos.
	/// \see set()
	void set(int startAddress, std::vector<char> values);

	/// Altera valores da tabela de controle C++ Style orientado a word (2 bytes).
	/// Versão mais simples de ser usada. O tamanho (length) é tamanho dos dados contidos no vector em bytes
	/// \param startAddress Endereço da tabela de controle do motor inicial para escrita
	/// \param values Valores para serem escritos.
	/// \see set()
	void set(int startAddress, std::vector<short int> values);

	/// Lê valores da tabela de controle C Style.
	/// Esta versão foi criada por motivos históricos, semelhante ao código atual da EDROM. Possui um estilo mais C.
	/// \param startAddress Endereço da tabela de controle do motor inicial para leitura
	/// \param length Número de bytes a serem lidos a partir de startAddress
	/// \param outValuesPtrPonteiro para os dados de saída. A função considera que esteja devidamente alocado e de acordo com o tamanho (length passado).O usuário deve tomar cuidado com o gerenciamento de memória
	/// \see get()
	void get(int startAddress, int length, char* outValuesPtr);

	/// Lê valores da tabela de controle C++ Style orientado a byte.
	/// Versão mais simples de ser usada. Retorna vector organizados em bytes.
	/// \param startAddress Endereço da tabela de controle do motor inicial para escrita
	/// \param length Número de bytes a serem lidos a partir de startAddress
	/// \see get()
	/// \return Vector de bytes contendo as informações lidas
	std::vector<char> getBytes(int startAddress, int nBytes);

	/// Lê valores da tabela de controle C++ Style orientado a word (2 bytes).
	/// Versão mais simples de ser usada. Retorna vector organizados em words (2 bytes).
	/// \param startAddress Endereço da tabela de controle do motor inicial para escrita
	/// \param length Número de words a serem lidos a partir de startAddress
	/// \see get()
	/// \return Vector de words contendo as informações lidas
	std::vector<short int> getWords(int startAddress, int nWords);

	/// Retorna uma string que mostra toda a memória do motor
	/// \todo Também mostrar o nome do campo
	std::string toString();

	/// Retorna uma string que mostra parte da memória do motor
	/// \param startAddress Endereço da tabela de controle do motor inicial para leitura
	/// \param length Número de bytes a serem lidos a partir de startAddress
	/// \todo Também mostrar o nome do campo
	std::string toString(int startAddress, int length);

};

#endif // DXLMEMMAP_H
