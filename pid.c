
/* * * * * * * * * * * * *  
 *                       *
 *   Self Balancing      *
 *       Robot           *
 *    with mpu6050       *
 *                       *
 * * * * * * * * * * * * */     


/* TODO: Notes
 *
 * b,o,g, third from bottom right (programmer)
 * 
 *
 *
 * */

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


#define NOKIA_VOP_VALUE 0x20                            
#define PRESCALE_VAL    1024                            
#define RAD_TO_DEG      180/3.14
#define GYRO_SEN        131.0
#define CMP_VAL         30                              // ISR interrupt freq = CMP_VAL/7812.5 
#define HPF             0.98                            // Ratio of Angle from gyro
#define LPF             (1 - HPF)                       // Ratio of Angle from accelerometer
#define INT_P           CMP_VAL/(F_CPU/PRESCALE_VAL)    // CMP_VAL = 100 -> 0.0128

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

#define PWM_OFFSET      152.0
#define PWM_SCALER      (255.0-PWM_OFFSET)

#define DEBUGG 0


char b[] = "Hello Wold";
char e[] = "Error";
char s[] = "Success";
volatile double angle;
volatile double gAngle;
volatile double cAngle;
volatile uint8_t first = 1;
volatile int16_t isrTime;
uint8_t error = 0;
double u;                       // MOVE 

void errorHandler(uint8_t e);
void init();
void nWrite(char *mess);
ISR(TIMER1_COMPA_vect);


/* ----------------------------*
 *      MAIN FUNCTION          *
 * ----------------------------*/

int main(void){
  init();
  TCNT1 = 0;
  sei();                                    // Enable global interrupts
  int32_t i = 0;

  while(1){

    if(DEBUGG){
      i++;
      if (i > 5000){
        i++;
        NOKIA_clear();
        sprintf(b ,"AG: %d", (int16_t)(gAngle));
        NOKIA_print(0, 0, b, 0); 
        sprintf(b ,"AA: %d", (int16_t)(angle));
        NOKIA_print(0, 9, b, 0); 
        sprintf(b ,"CA: %d", (int16_t)(cAngle));
        NOKIA_print(0, 18, b, 0); 
        sprintf(b ,"U: %d", (int)(u));
        NOKIA_print(0, 27, b, 0); 
        NOKIA_update();
        i = 0;
      }
    }

  }
}

/* ----------------------------*
 *      Nokia Write            *
 * ----------------------------*/

void nWrite(char *mess){

  /* NOKIA_print(0, 0, mess, 0); */ 
  /* NOKIA_update(); */
  /* NOKIA_scroll(-9); */

}

/* ----------------------------*
 *      Error Handler          *
 * ----------------------------*/

void errorHandler(uint8_t err){
  if(err){

    /* NOKIA_print(0, 0, e, 0); */
    /* NOKIA_update(); */
    /* while(1); */

  }else{

    /* NOKIA_print(0, 0, s, 0); */
    /* NOKIA_update(); */

  }
}

/* ----------------------------*
 *          INIT               *
 * ----------------------------*/

void init(){
  // Init timed interrupt 
  cli();                                    // Disable global interrupts
  TIMSK1 |= (1 << OCIE1A);                  // Compare A interrupt enable
  OCR1A = CMP_VAL;                          // Compare value
  TCCR1B |= ((1 << CS12) | (1 << CS10));    // Prescaleler = 1024

  // Phase-correct PWM 
  DDRD |= (1 << DDD6);                      // PWM on PD6
  DDRD |= (1 << DDD7);                      // Direction control
  DDRB |= (1 << DDB0);                      // Direction control
  OCR0A = 0x0;                              // Duty cycle 0%
  TCCR0A |= (1 << COM0A1) | (0 << COM0A0);  // Set on counting up, clear on count down
  TCCR0A |= (1 << WGM02)  | (1 << WGM00);   // Phase-correct, TOP: OCRA
  TCCR0B |= (1 << CS00);                    // No prescaling

  /* // Init nokia display */
  /* NOKIA_init(0); */
  /* NOKIA_LED_ENABLE(); */
  /* NOKIA_setVop(NOKIA_VOP_VALUE); */
  /* NOKIA_update(); */
  /* NOKIA_LED_PORT |= (1 << NOKIA_LED); */
  
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
  i2c_write(SCALE_3);
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

/* ----------------------------*
 *          INTERRUPT          *
 * ----------------------------*/

ISR(TIMER1_COMPA_vect){

  cli();
  TCNT1= 0;

  // PID
  double Kp = 7.0;
  double Ki = 18;
  double Kd = 0;
  double r  = 0;
  double e = 0;
  static double eOld;
  static double ei;
  

  // Gyroscope vars
  unsigned char gxh, gxl;
  int16_t gx;
  double nameHolder;
  
  // Accelerometer vars
  unsigned char yh, yl, zh, zl, t;
  int16_t y, z;

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
  y     = ((int16_t)(yh) << 8) + (int16_t)(yl);
  z     = ((int16_t)(zh) << 8) + (int16_t)(zl);
  angle = atan((double)(y)/(double)(z)); 
  angle = angle * RAD_TO_DEG; 

  // Angle from gyroscope
  gx = ((int16_t)(gxh) << 8) + (int16_t)(gxl);
  gx = ((gx + 866) / GYRO_SEN);              // shift: SCALE_0: ~ 131 SCALE_3: ~ 16
  /* gx = ((gx + 866)  >> 7); */             // Faster implementation but division by 132
  nameHolder = ((double)(gx))*INT_P;  

  if(first == 1){
    nameHolder = angle;
    first = 0;
  }

  gAngle  = gAngle + nameHolder; 
  cAngle  = HPF * (cAngle + nameHolder) + LPF * angle;

  // PID  - Add direction and map u-value range from 0 to 255
  e    = r - cAngle;
  ei   = ei + e * INT_P; 
  u    = e*Kp + ei*Ki + (e-eOld)*INT_P*Kd;
  eOld = e;

   if( ei > 90)
    ei = 90;
  else if (ei < -90)
    ei = 90;


  // Stop overflow in timer register
  if (u > 90)
    u = 90;
  else if (u < -90)
    u = -90;

  // map u between 0 and 255
  u = (PWM_SCALER/90.0)*u;

  if(u < 0)
    u -= PWM_OFFSET;
  if(u > 0)
    u += PWM_OFFSET;

  if( u < 0){
    PORTD |= (1 << DDD7);
    PORTB &= ~(1 << DDB0);
  }
  else{
    PORTB |= (1 << DDB0);
    PORTD &= ~(1 << DDD7);
  }

  OCR0A = (int)(abs(u));
  /* OCR0A = 255; */

  sei(); 
  isrTime = TCNT1;
}
