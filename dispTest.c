#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* #include <avr/pgmspace.h> */
/* #include <avr/interrupt.h> */
/* #include <avr/sleep.h> */
#include "nokia5110.h"    

char b[] = "Hello Wold";

int main(void){

NOKIA_init(0);
NOKIA_LED_ENABLE();
NOKIA_setVop(0x20);
NOKIA_print(0, 0, b, 0);
NOKIA_update();
NOKIA_LED_PORT |= (1 << NOKIA_LED);
while(1);

}
