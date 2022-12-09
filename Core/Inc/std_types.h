#ifndef STD_TYPES_H
#define STD_TYPES_H
typedef enum {
    OP_INVALID = 0xFF,
    OP_LED_ON = 1,
    OP_LED_OFF = 2,
    OP_START = 3,
    OP_STOP = 4,
    OP_BAUD = 5,
    OP_WORLD_LENGTH = 6
} operationType;



typedef enum {
    STATE_INVALID = 0xFF,
    STATE_LED_ON_ONGOING = 1,
    STATE_LED_ON_INITIAL = 2,
    STATE_LED_OFF_ONGOING = 3,
    STATE_LED_OFF_INITIAL = 4
} ledOperationsStateType;
#endif