#include "stringOp.h"



void echoFunc(uint8_t* data) {
    printf("%s\n", data);
	return;
}

void strOp(uint8_t* input) {
    //char buf[] ="abc/qwe/ccd";
    int i = 0;
    char *p = strtok (input, "=");
    char *array[2];

    while (p != NULL)
    {
        array[i++] = p;
        p = strtok (NULL, "/");
    }

    for (i = 0; i < 3; ++i) 
        printf("%s\n", array[i]);

    return;
}
