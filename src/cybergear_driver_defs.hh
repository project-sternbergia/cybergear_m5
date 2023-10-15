#ifndef CYBER_GEAR_DRIVER_DEFS_H
#define CYBER_GEAR_DRIVER_DEFS_H

#define CMD_POSITION                    1
#define CMD_RESPONSE                    2
#define CMD_ENABLE                      3
#define CMD_RESET                       4
#define CMD_SET_MECH_POSITION_TO_ZERO   6
#define CMD_CHANGE_CAN_ID               7
#define CMD_RAM_READ                    17
#define CMD_RAM_WRITE                   18
#define CMD_GET_MOTOR_FAIL              21

#define ADDR_RUN_MODE              0x7005
#define ADDR_IQ_REF                0x7006
#define ADDR_SPEED_REF             0x700A
#define ADDR_LIMIT_TORQUE          0x700B
#define ADDR_CURRENT_KP            0x7010
#define ADDR_CURRENT_KI            0x7011
#define ADDR_CURRENT_FILTER_GAIN   0x7014
#define ADDR_LOC_REF               0x7016
#define ADDR_LIMIT_SPEED           0x7017
#define ADDR_LIMIT_CURRENT         0x7018

#define MODE_MOTION   0x00
#define MODE_POSITION 0x01
#define MODE_SPEED    0x02
#define MODE_CURRENT  0x03

#define P_MIN     -12.5f
#define P_MAX      12.5f
#define V_MIN     -30.0f
#define V_MAX      30.0f
#define KP_MIN      0.0f
#define KP_MAX    500.0f
#define KD_MIN      0.0f
#define KD_MAX      5.0f
#define T_MIN     -12.0f
#define T_MAX      12.0f
#define IQ_MIN     -27.0f
#define IQ_MAX      27.0f
#define CURRENT_FILTER_GAIN_MIN 0.0f
#define CURRENT_FILTER_GAIN_MAX 1.0f

#define DEFAULT_CURRENT_KP           0.125f
#define DEFAULT_CURRENT_KI           0.0158f
#define DEFAULT_CURRENT_FINTER_GAIN  0.1f

#define RET_CYBERGEAR_OK              0x00
#define RET_CYBERGEAR_MSG_NOT_AVAIL   0x01
#define RET_CYBERGEAR_INVALID_CAN_ID  0x02
#define RET_CYBERGEAR_INVALID_PACKET  0x03


#endif // !CYBER_GEAR_DRIVER_DEFS_H
