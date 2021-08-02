#include <xc.h>
#include <math.h>
#include "color.h"
#include "motor.h"
#include "i2c.h"
void I2C_2_Master_Init(void)
{
  //i2c config  
  SSP2CON1bits.SSPM= 0b1000;    // i2c master mode
  SSP2CON1bits.SSPEN = 1;       //enable i2c
  SSP2ADD = (_XTAL_FREQ/(4*_I2C_CLOCK))-1; //Baud rate divider bits (in master mode)
  
  //pin configuration for i2c  
  TRISDbits.TRISD5 = 1;                   //Disable output driver
  TRISDbits.TRISD6 = 1;                   //Disable output driver
  ANSELDbits.ANSELD5=0;
  ANSELDbits.ANSELD6=0;
  SSP2DATPPS=0x1D;      //pin RD5
  SSP2CLKPPS=0x1E;      //pin RD6
  RD5PPS=0x1C;      // data output
  RD6PPS=0x1B;      //clock output  
}

void I2C_2_Master_Idle(void)
{
  while ((SSP2STAT & 0x04) || (SSP2CON2 & 0x1F)); // wait until bus is idle
}

void I2C_2_Master_Start(void)
{
  I2C_2_Master_Idle();    
  SSP2CON2bits.SEN = 1;             //Initiate start condition
}

void I2C_2_Master_RepStart(void)
{
  I2C_2_Master_Idle();
  SSP2CON2bits.RSEN = 1;           //Initiate repeated start condition
}

void I2C_2_Master_Stop()
{
  I2C_2_Master_Idle();
  SSP2CON2bits.PEN = 1;           //Initiate stop condition
}

void I2C_2_Master_Write(unsigned char data_byte)
{
  I2C_2_Master_Idle();
  SSP2BUF = data_byte;         //Write data to SSPBUF
}

unsigned char I2C_2_Master_Read(unsigned char ack)
{
  unsigned char tmp;
  I2C_2_Master_Idle();
  SSP2CON2bits.RCEN = 1;        // put the module into receive mode
  I2C_2_Master_Idle();
  tmp = SSP2BUF;                //Read data from SS2PBUF
  I2C_2_Master_Idle();
  SSP2CON2bits.ACKDT = !ack;     // 0 turns on acknowledge data bit
  SSP2CON2bits.ACKEN = 1;        //start acknowledge sequence
  return tmp;
}

float PWR(int Base, float n) {
    float Binomial = (1 + n*(Base - 1) + (n*(n-1)*(Base - 1)*(Base - 1))/2);
    return Binomial;
}

float arccos(float x) {
    float acos = (3/4 - x - (x*x*x)/6);
    return acos;
}

unsigned int RGBtoHue(int Red, int Green, int Blue){
    float hue;
    float total = Red + Blue + Green;
    float R = Red/total;
    float G = Green/total;
    float B = Blue/total;
    //float X = (2*R) - G - B;
    //float Y = (R-G)*(R-G);
    //float Z = (R - B)*(G - B);
    //float W = sqrt(Y+Z);
    hue = acos(((2*R) - G - B)/(2*sqrt(((R-G)*(R-G))+((R - B)*(G - B))))) * (360/(2*3.141593));
    //hue = arccos(((2*R) - G - B)/(2*PWR((((R-G)*(R-G))+((R - B)*(G - B))), 0.5)) * (360/(2*3.141593)));
    
    if (B > G) {
        hue = 360 - hue;
    }
    return hue;
}
   
void LED_interrupt_init(void)
{
    T0CON1bits.T0CS=0b010; // Fosc/4
    T0CON1bits.T0ASYNC=1; // see datasheet errata - needed to ensure correct operation when Fosc/4 used as clock source
    T0CON1bits.T0CKPS=0b0100; // need to work out prescaler to produce a timer tick corresponding to 1 deg angle change
    T0CON0bits.T016BIT=1;	//16bit mode	
	
    // it's a good idea to initialise the timer so that it initially overflows after 20 ms
    TMR0H=0b00001101;
    TMR0L=0b11010000;
    T0CON0bits.T0EN=1;	//start the timer
    
    PIE0bits.TMR0IE = 1; //Timer0 interrupt enable
    IPR0bits.TMR0IP = 0; //Sets interrupt priority for timer to  high
    INTCONbits.IPEN = 1; //Enables interrupt priority levels
    INTCONbits.GIE=1;   //Enable global interrupts
    INTCONbits.PEIE = 1; // Enables peripheral interrupts
}

void write16bitTMR0val(unsigned int tmp)
{
    TMR0H=tmp>>8; //MSB written
    TMR0L=tmp; //LSB written and timer updated
}
