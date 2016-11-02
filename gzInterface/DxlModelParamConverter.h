#ifndef DXLMODELPARAMCONVERTER_H
#define DXLMODELPARAMCONVERTER_H

class DxlModelParamConverter {
	
	
	private:

	int model = 0; /**< Número do modelo */

	double posResolution = 0; /**< Resolução de Posição. Em Radianos por Unidade de motor */

	double angleRange = 0; /**< Intervalo de ângulo no modo Joint */

	double velResolution = 0; /**< Resolução de Velocidade. Em Radianos/s por Unidade de motor */

	double torqueResolution = 0; /**< Resolução de torque em N.m */

	public:


	DxlModelParamConverter();


	int getModel(){return model;}

	double getPosResolution(){return posResolution;}

	double getAngleRange(){return angleRange;}

	double getVelResolution(){return velResolution;}

	double getTorqueResolution(){return torqueResolution;}


	/**
	 * @brief Construtor que lê parâmetros de um arquivo json
	 *
	 * @param jsonFileName Caminho para o arquivo json onde estão as configurações desejadas
	 */
	DxlModelParamConverter(const char* jsonFileName);

	/**
	 * @brief Converte posição em unidade de motor para radianos
	 *
	 * @param pos Posição em unidade de motor
	 * @return double Posição em radianos
	 */
	double posToRad(short int pos);

	/**
	 * @brief Converte velocidade em unidade de motor para radianos por segundo
	 *
	 * @param vel Velocidade em unidade de motor
	 * @return double Velocidade em radianos por segundo
	 */
	double velToRad(short int vel);

	/**
	 * @brief Converte torque em unidade de motor para N.m (Newton metro)
	 *
	 * @param torque Torque em unidade de motor
	 * @return double Torque em N.m
	 */
	double torqueToNm(short int torque);


};

#endif //DXLMODELPARAMCONVERTER_H
