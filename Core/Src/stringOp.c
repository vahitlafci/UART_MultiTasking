#include "stringOp.h"
#include "std_types.h"
/*shared variables*/
extern operationType current_operation;
extern uint16_t ledOnTime;
extern uint16_t ledOffTime;
extern uint32_t baud;
extern uint8_t ledEvent;
extern uint8_t uartEvent;
/*static variables*/
static uint16_t ledOnTime_previous;
static uint16_t ledOffTime_previous;

void strOp(uint8_t *input)
{
    int i = 0;
    char *p = strtok(input, "=");
    char *array[2];

    while (p != NULL)
    {
        array[i++] = p;
        p = strtok(NULL, "/");
    }

    if (strstr(array[0], "ledon"))
    {
        current_operation = OP_LED_ON;
        ledEvent = 1;
        ledOnTime = atoi(array[1]);
    }
    else if (strstr(array[0], "ledoff"))
    {
        current_operation = OP_LED_OFF;
        ledEvent = 1;
        ledOffTime = atoi(array[1]);
    }
    else if (strstr(array[0], "start"))
    {
        current_operation = OP_START;
        ledEvent = 1;
        ledOnTime = ledOnTime_previous;
        ledOffTime = ledOffTime_previous;
    }
    else if (strstr(array[0], "stop"))
    {
        current_operation = OP_STOP;
        ledEvent = 1;
        ledOffTime_previous = ledOffTime;
        ledOnTime_previous = ledOnTime;
        ledOnTime = 1000;
        ledOffTime = 1000; 
    }
    else if (strstr(array[0], "baud"))
    {
        current_operation = OP_BAUD;
        uartEvent = 1;
        baud = atoi(array[1]);
    }
    else if (strstr(array[0], "wordlength"))
    {
        //current_operation = OP_WORLD_LENGTH;  TODO(VahitL)
    }
    else
    {
        current_operation = OP_INVALID;
    }
    return;
}
