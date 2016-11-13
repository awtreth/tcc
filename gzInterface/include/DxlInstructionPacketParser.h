#ifndef DXLINSTRUCTIONPACKETPARSER_H
#define DXLINSTRUCTIONPACKETPARSER_H

#include "DxlModelParamConverter.h"


/**
 * @brief
 *
 */
class DxlInstructionPacketParser {

    public:

    /**
     * @brief
     *
     */
    enum BasicInstructionType {READ, WRITE, OTHER, NONE};



    private:


    gz_msgs::DxlInstructionPacket packet; /**< TODO: describe */


    BasicInstructionType basicInstructionType; /**< TODO: describe */


    public:

    /**
         * @brief
         *
         */
    //        enum BasicInstructionType {READ, WRITE, OTHER, NONE};


    /**
         * @brief
         *
         */
    DxlInstructionPacketParser();

    /**
         * @brief
         *
         * @param _packet
         */
    DxlInstructionPacketParser(gz_msgs::DxlInstructionPacket _packet);


    /**
         * @brief
         *
         * @param _packet
         * @return bool
         */
    bool loadDxlInstructionPacket(gz_msgs::DxlInstructionPacket _packet);

    /**
         * @brief
         *
         * @return gz_msgs::DxlInstructionPacket::InstructionType
         */
    gz_msgs::DxlInstructionPacket::InstructionType getInstructionType();


    /**
//	 * @brief
//	 *
//	 * @param addr
//	 * @param size
//	 * @return bool
//	 */
    //	bool hasParameter(int addr, int size);


    //	char getByte(int addr, int id);

    //	short getWord(int addr, int id);

    //	void getParameter(int addr, int length, char* output, int id);


    //	/**
    //	 * @brief
    //	 *
    //	 * @return bool
    //	 */
    //	bool hasPos();

    //	/**
    //	 * @brief
    //	 *
    //	 * @return bool
    //	 */
    //	bool hasVel();

    //	/**
    //	 * @brief
    //	 *
    //	 * @return bool
    //	 */
    //	bool hasTorque();

    //	/**
    //	 * @brief
    //	 *
    //	 * @return short
    //	 */
    //	short int getPos(int id = 0);

    //	/**
    //	 * @brief
    //	 *
    //	 * @return short
    //	 */
    //	short int getVel(int id = 0);

    //	/**
    //	 * @brief
    //	 *
    //	 * @return short
    //	 */
    //	short int getTorque(int id = 0);

};


#endif // DXLINSTRUCTIONPACKETPARSER_H
