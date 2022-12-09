#include "stringOp.h"


static uint8_t result = 0xFF;
static uint8_t val = 0;
void echoFunc(uint8_t* data) {
    printf("%s\n", data);
	return;
}

void strOp(uint8_t* input) {
    int i = 0;
    char *p = strtok (input, "=");
    char *array[2];

    while (p != NULL)
    {
        array[i++] = p;
        p = strtok (NULL, "/");
    }
    val = 0;

    if(strstr(array[0],"ledon")) {
        result = 1;
    	val = atoi(array[1]);
    } else if(strstr(array[0],"ledoff")) {
        result = 2;
    	val = atoi(array[1]);
    } else if(strstr(array[0],"start")) {
    	result = 3;
    } else if(strstr(array[0],"stop")) {
    	result = 4;
    } else if(strstr(array[0],"baud")) {
    	result = 5;
    	val = atoi(array[1]);
    } else {
    	result = 0xFF;
    }
    return;
}
