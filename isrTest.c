#define F_CPU 1000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "nokia5110.h"    
/* #include "i2cmaster.h" */

#define CMP_VAL 1000
#define NOKIA_VOP_VALUE 0x20

char b[] = "Hello Wold";
volatile int i = 0;

void init();
ISR(TIMER1_COMPA_vect);

int main(void){

  init();

  sprintf(b, "%d         ", i);
  NOKIA_print(0, 0, b, 0);
  NOKIA_update();
  TCNT1 = 0;
  sei();                                    // Enable global interrupts
  while(1);
}


void init(){
  //init timed interrupt ~1 Hz
  cli();                                    // Disable global interrupts
  TIMSK1 |= (1 << OCIE1A);                  // Compare A interrupt enable
  OCR1A = CMP_VAL;                          // Compare value
  TCCR1B |= ((1 << CS12) | (1 << CS10));    // Prescaleler = 1024


  //Init nokia display
  NOKIA_init(0);
  NOKIA_LED_ENABLE();
  NOKIA_setVop(NOKIA_VOP_VALUE);
  NOKIA_print(0, 0, b, 0);
  NOKIA_update();
  NOKIA_LED_PORT |= (1 << NOKIA_LED);
}

ISR(TIMER1_COMPA_vect){
    i++;
    sprintf(b, "%d ", i);
    NOKIA_print(0, 0, b, 0);
    NOKIA_update();
    TCNT1 = 0;
}
