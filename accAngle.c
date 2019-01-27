
/* * * * * * * * * * * * *  
 *                       *
 * I2C test for mpu6050  *
 *                       *
 * * * * * * * * * * * * */     

#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "nokia5110.h"    
#include "i2cmaster.h"
#include <math.h>

#define CMP_VAL 16000
#define NOKIA_VOP_VALUE 0x20

/* MPU6050 */
#define MPU_ADDR    0x68 * 2
#define MPU_PWR     0x6B
#define ACC_CONF    0x1c
#define GYRO_CONF   0x1B
#define ACC_XH      0x3B
#define ACC_XL      0x3C
#define ACC_YH      0x3D
#define ACC_YL      0x3E
#define ACC_ZH      0x3F
#define ACC_ZL      0x40


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
  i2c_start_wait(MPU_ADDR);               // Write to MPU
  i2c_write(MPU_PWR);               // write to pwr management
  errorHandler(error);                      // check for error 
  error = i2c_write(0x0);                   // write 0, for internal clk source
  errorHandler(error);
  i2c_stop();

  i2c_start_wait(MPU_ADDR);
  i2c_write(ACC_CONF);
  i2c_write(0x18);
  i2c_stop();
}

/* -----------------------------
 *          INTERRUPT
 * ----------------------------*/

ISR(TIMER1_COMPA_vect){

  /* cli(); */
  TCNT1= 0;
  
  char mess [20];
  unsigned char yh, yl, zh, zl;
  int y,z;
  double angle;

  i2c_start_wait(MPU_ADDR);
  i2c_write(ACC_YH);
  i2c_rep_start(MPU_ADDR + I2C_READ);
   
  yh = i2c_readAck(); 
  yl = i2c_readAck(); 
  zh = i2c_readAck(); 
  zl = i2c_readAck(); 
  
  i2c_stop();

  y = ((int)(yh) << 8) + yl;
  z = ((int)(zh) << 8) + zl;
  
  angle = atan((double)(y)/(double)(z)); 
  angle = angle * 180.0/3.14;

  sprintf(mess,"A: %d", (int)(angle)); 

  NOKIA_scroll(-50);
  NOKIA_print(0, 0, mess, 0); 
  NOKIA_update();
  NOKIA_scroll(-9);
  TCNT1 = 0;
  /* sei(); */

}
