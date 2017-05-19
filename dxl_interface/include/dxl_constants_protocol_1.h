#ifndef DXL_CONSTANTS_PROTOCOL_1_H
#define DXL_CONSTANTS_PROTOCOL_1_H

/*
 * CONTROL TABLE PROTOCOL 1.0
 */

#define P1_CONTROL_TABLE_SIZE 75

//EEPROM
#define P1_MODEL_NUMBER_L	0
#define P1_MODEL_NUMBER_H	1
#define P1_VERSION_FIRMWARE	2
#define P1_ID                3
#define P1_BAUD_RATE         4
#define P1_RETURN_DELAY_TIME	5
#define P1_CW_ANGLE_LIMIT_L	6
#define P1_CW_ANGLE_LIMIT_H	7
#define P1_CCW_ANGLE_LIMIT_L	8
#define P1_CCW_ANGLE_LIMIT_H	9
#define P1_DRIVE_MODE        10

#define P1_LIMIT_TEMPERATURE     11
#define P1_LOW_LIMIT_VOLTAGE     12
#define P1_HIGH_LIMIT_VOLTAGE    13
#define P1_MAX_TORQUE_L          14
#define P1_MAX_TORQUE_H          15
#define P1_RETURN_LEVEL          16
#define P1_ALARM_LED             17
#define P1_ALARM_SHUTDOWN        18

//MX ONLY
#define P1_MULTI_TURN_OFFSET_L   20
#define P1_MULTI_TURN_OFFSET_H   21
#define P1_RESOLUTION_DIVIDER    22

//RAM
#define P1_TORQUE_ENABLE         24
#define P1_LED                   25

//EX, AX, RX and DX Series
#define P1_CW_COMPLIANCE_MARGIN	26
#define P1_CCW_COMPLIANCE_MARGIN	27
#define P1_CW_COMPLIANCE_SLOPE	28
#define P1_CCW_COMPLIANCE_SLOPE	29

//MX ONLY
#define P1_D_GAIN		26
#define P1_I_GAIN		27
#define P1_P_GAIN		28

#define P1_GOAL_POSITION_L       30
#define P1_GOAL_POSITION_H       31
#define P1_GOAL_SPEED_L          32
#define P1_GOAL_SPEED_H          33
#define P1_TORQUE_LIMIT_L        34
#define P1_TORQUE_LIMIT_H        35

#define P1_PRESENT_POSITION_L	36
#define P1_PRESENT_POSITION_H	37
#define P1_PRESENT_SPEED_L       38
#define P1_PRESENT_SPEED_H       39
#define P1_PRESENT_LOAD_L        40
#define P1_PRESENT_LOAD_H        41
#define P1_PRESENT_VOLTAGE       42
#define P1_PRESENT_TEMPERATURE	43

#define P1_REGISTERED        44

#define P1_MOVING            46
#define P1_LOCK          	47
#define P1_PUNCH_L       	48
#define P1_PUNCH_H   		49

//EX106 ONLY
#define P1_SENSED_CURRENT_L  56
#define P1_SENSED_CURRENT_H  57

//MX106 ONLY
#define P1_CURRENT_L             68
#define P1_CURRENT_H             69
#define P1_TORQUE_MODE_ENABLE    70
#define P1_GOAL_TORQUE_L         71
#define P1_GOAL_TORQUE_H         72
#define P1_GOAL_ACCELERATION     73

#endif
