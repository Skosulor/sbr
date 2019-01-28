
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
#define RAD_TO_DEG      180/3.14
#define GYRO_SEN        131.0

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
volatile double gAngle;
volatile double cAngle;
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
        sprintf(b ,"AG: %d", (int)(gAngle));
        NOKIA_print(0, 0, b, 0); 
        sprintf(b ,"AA: %d", (int)(angle));
        NOKIA_print(0, 9, b, 0); 
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

  // Gyroscope vars
  unsigned char gxh, gxl;
  int gx;
  
  // Accelerometer vars
  unsigned char yh, yl, zh, zl, t;
  int y, z;

  // Get mpu Values
  i2c_start_wait(MPU_ADDR + I2C_WRITE);
  i2c_write(ACC_YH);
  i2c_rep_start(MPU_ADDR + I2C_READ);

  // Accelerometer
  yh = i2c_readAck(); 
  yl = i2c_readAck(); 
  zh = i2c_readAck(); 
  zl = i2c_readAck(); 

  // Temprature
  t = i2c_readAck(); 
  t = i2c_readAck(); 

  // Gyro
  gxh = i2c_readAck(); 
  gxl = i2c_readNak(); 
  
  // Stop i2c communication
  i2c_stop();

  // Angle from Accelerometer
  y = ((int)(yh) << 8) + (int)(yl);
  z = ((int)(zh) << 8) + (int)(zl);
  angle = atan((double)(y)/(double)(z)); 
  angle = angle * RAD_TO_DEG; 

  // Angle from gyroscope
  gx = ((int)(gxh) << 8) + (int)(gxl);
  gx = ((gx + 866) / GYRO_SEN);              // shift: SCALE_0: ~ 131 SCALE_3: ~ 16
  gAngle = gAngle + ((double)(gx))*INT_PERIOD;  

  sei(); 

}
