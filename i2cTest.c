/*  I2C test for mpu6050
 *  
 *  == Sample Rate Divider ==
 *  8 bit register R/W
 *  Address: 0x19
 *  Sample rate = 8kHz/(1 + SRD) 
 *
 *  == Gyroscope Configuration ==
 *  8 bit register R/W
 *  Used for self-test and scale range 
 *  Bit 7,6,5: X,Y,Z self-test
 *  bit 3-4: scale range
 *  Address: 0x1B
 *
 *  == Accelerometer Configuration ==
 *  8 bit register R/W
 *  Used for self-test and scale range 
 *  Bit 7,6,5: X,Y,Z self-test
 *  bit 3-4: scale range (± 2g, 4g, 8g16g)
 *  Address: 0x1C
 *
 *  == FIFO Enable ==
 *  8 bit register R/W
 *  Enables sensor sampling, each axis (gyro) needs individual enabling
 *  Bit 6-4: Gyro Enable X,Y,Z
 *  Bit 3: Accelerometer Enable
 *  Bit 0-2: Exernal slave
 *
 *  == I2C Master Control ==
 *  8 bit register R/W
 *
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "nokia5110.h"    
#include "i2cmaster.h"

#define CMP_VAL 8000
#define NOKIA_VOP_VALUE 0x20

#define MPU_ADDR 0x68 * 2
#define MPU_PWR 0x6B

char b[] = "Hello Wold";
char e[] = "Error";
char s[] = "Success";
volatile int i = 0;
uint8_t error = 0;

void errorHandler(uint8_t e);
void init();
void nWrite(char *mess);
ISR(TIMER1_COMPA_vect);


/* -----------------------------
 *      MAIN FUNCTION
 * ----------------------------*/

int main(void){

  init();

  /* sprintf(b, "%d         ", i); */
  /* NOKIA_print(0, 0, b, 0); */
  /* NOKIA_update(); */
  TCNT1 = 0;
  sei();                                    // Enable global interrupts
  /* nWrite("Main"); */
  while(1);
}

/* -----------------------------
 *      Nokia Write
 * ----------------------------*/

void nWrite(char *mess){
  NOKIA_print(0, 0, mess, 0); 
  NOKIA_update();
  NOKIA_scroll(-9);

}

/* -----------------------------
 *      Error Handler
 * ----------------------------*/

void errorHandler(uint8_t err){
  if(err){
    NOKIA_print(0, 0, e, 0);
    NOKIA_update();
    while(1);
  }else{
    NOKIA_print(0, 0, s, 0);
    NOKIA_update();
  }
}

/* -----------------------------
 *          INIT
 * ----------------------------*/

void init(){
  // init timed interrupt ~1 Hz
  cli();                                    // Disable global interrupts
  TIMSK1 |= (1 << OCIE1A);                  // Compare A interrupt enable
  OCR1A = CMP_VAL;                          // Compare value
  TCCR1B |= ((1 << CS12) | (1 << CS10));    // Prescaleler = 1024


  // Init nokia display
  NOKIA_init(0);
  NOKIA_LED_ENABLE();
  NOKIA_setVop(NOKIA_VOP_VALUE);
  /* NOKIA_print(0, 0, b, 0); */
  NOKIA_update();
  NOKIA_LED_PORT |= (1 << NOKIA_LED);
  
  // mpu6050 setup
  i2c_init();
  /* nWrite("Send Start seq.."); */
  i2c_start_wait(MPU_ADDR);               // Write to MPU

  /* nWrite("Start seq done"); */
  /* error = */
  i2c_write(MPU_PWR);               // write to pwr management
  /* nWrite("write to addr"); */
  errorHandler(error);                      // check for error 
  error = i2c_write(0x0);                   // write 0, for internal clk source
  errorHandler(error);
  i2c_stop();
}

/* -----------------------------
 *          INTERRUPT
 * ----------------------------*/

ISR(TIMER1_COMPA_vect){
    char mess [20];
    unsigned char r;
    uint8_t t = 0;
    i++;
    i2c_start_wait(MPU_ADDR);
    i2c_write(0x3B);
    i2c_rep_start(MPU_ADDR + I2C_READ);
    r = i2c_readNak(); 
    i2c_stop();
    t = (int)(r);

    sprintf(mess, "%d", t);
    /* NOKIA_print(0, 0, mess, 0); */
    /* NOKIA_update(); */
    nWrite(mess);
    TCNT1 = 0;

}
