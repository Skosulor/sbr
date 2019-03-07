
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
#include <util/delay.h>

#define CMP_VAL 100           // ISR interrupt freq = CMP_VAL/7812.5 : 100 ~ 12.8 ms = 78 Hz
#define INT_PERIOD 0.0128     // 1/78.125
#define NOKIA_VOP_VALUE 0x20
#define RAW 1

/* MPU6050 */
#define MPU_ADDR        (0x68 << 1)
#define MPU_PWR         0x6B
#define ACC_CONF        0x1c
#define GYRO_CONF       0x1B
#define CONF            0x1A

#define ACC_XH          0x3B
#define ACC_XL          0x3C
#define ACC_YH          0x3D
#define ACC_YL          0x3E
#define ACC_ZH          0x3F
#define ACC_ZL          0x40

#define GYRO_XH         0x43
#define GYRO_XL         0x44
#define GYRO_YH         0x45
#define GYRO_YL         0x46
#define GYRO_ZH         0x47
#define GYRO_ZL         0x48

#define SCALE_0         0x00    // 2g .. 250°/s
#define SCALE_1         0x08    // 4g .. 500°/s
#define SCALE_2         0x10    // 8g .. 1000°/s
#define SCALE_3         0x18    // 16g .. 2000°/s

#define INTERNAL_CLK    0x00

#define NO_LP           0x00    // ~No lp filter
#define LP_1            0x01 
#define LP_2            0x02 
#define LP_3            0x03 
#define LP_4            0x04 
#define LP_5            0x05 
#define MAX_LP          0x06    // Max lp filter

#define DEBUGG 1


char b[] = "Hello Wold";
char e[] = "Error";
char s[] = "Success";
volatile int i = 0;
volatile double angle;
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
  TCNT1 = 0;
  sei();                                    // Enable global interrupts
  int i = 0;

  while(1){

    if(DEBUGG){
      i++;
      if (i > 10000){
        i++;
        NOKIA_clear();
        sprintf(b ,"A: %d", (int)(angle));
        NOKIA_print(0, 0, b, 0); 
        NOKIA_update();
        i = 0;
      }
    }

  }
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
  i2c_start_wait(MPU_ADDR);                 // Write to MPU
  i2c_write(MPU_PWR + I2C_WRITE);           // write to pwr management
  errorHandler(error);                      // check for error 
  error = i2c_write(0x0);                   // write 0, for internal clk source
  errorHandler(error);
  i2c_stop();

  // set accelerometer scale range
  i2c_start_wait(MPU_ADDR);
  i2c_write(ACC_CONF + I2C_WRITE);
  i2c_write(SCALE_0);
  i2c_stop();
  
  // set gyroscope scale range
  i2c_start_wait(MPU_ADDR);
  i2c_write(GYRO_CONF + I2C_WRITE);
  i2c_write(SCALE_0);
  i2c_stop();

  // Enable mpu low pass filter
  i2c_start_wait(MPU_ADDR);
  i2c_write(CONF + I2C_WRITE);
  i2c_write(MAX_LP);
  i2c_stop();
}

/* -----------------------------
 *          INTERRUPT
 * ----------------------------*/

ISR(TIMER1_COMPA_vect){

  cli();
  TCNT1= 0;
  char mess [20];
  unsigned char xh, xl;
  int x;
  
  /* static double angle; */

  // Get mpu Values
  i2c_start_wait(MPU_ADDR + I2C_WRITE);
  i2c_write(GYRO_XH);
  i2c_rep_start(MPU_ADDR + I2C_READ);
  // Gyro

  /* xl = i2c_readAck(); */ 
  /* xl = i2c_readAck(); */ 
  /* xl = i2c_readAck(); */ 
  /* xl = i2c_readAck(); */ 
  /* xl = i2c_readAck(); */ 
  /* xl = i2c_readAck(); */ 

  xh = i2c_readAck(); 
  xl = i2c_readNak(); 
  
  // Stop i2c communication
  i2c_stop();

  x = ((int)(xh) << 8) + (int)(xl);
  if(!RAW){
    x = ((x + 866) >> 7);                // SCALE_3: ~ 16
                                       // SCALE_0: ~ 131
    angle = angle + ((double)(x))*INT_PERIOD;  
  }else
    angle = x;

  sei(); 
}
