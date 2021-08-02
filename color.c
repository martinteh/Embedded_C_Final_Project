#include <xc.h>
#include "color.h"
#include "i2c.h"
#include "motor.h"
#include "serial.h"
#include <stdio.h>
#include <stdlib.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK INITIALISATION FUNCTION                                       //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void color_click_init(void)
{   
     //setup colour sensor via i2c interface
    I2C_2_Master_Init();      //Initialise i2c Master with 100KHz clock

     //set device PON
    color_writetoaddr(0x00, 1);
    __delay_ms(3); //need to wait 3ms for everthing to start up

    color_writetoaddr(0x00, 0b00011); //enable interrupts and turn on device ADC  

    //set integration time
    color_writetoaddr(0x01, 0xD5);
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK WRITE TO ADDRESS FUNCTION                                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void color_writetoaddr(char address, char value){
    I2C_2_Master_Start();         //Start condition
    I2C_2_Master_Write(0x52 | 0x00);     //7 bit device address + Write mode
    I2C_2_Master_Write(0x80 | address);    //command + register address
    I2C_2_Master_Write(value);    
    I2C_2_Master_Stop();          //Stop condition
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK READ RED FUNCTION                                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
unsigned int color_read_Red(void)
{
	unsigned int tmp;
	I2C_2_Master_Start();         //Start condition
	I2C_2_Master_Write(0x52 | 0x00);     //7 bit address + Write mode
	I2C_2_Master_Write(0xA0 | 0x16);    //command (auto-increment protocol transaction) + start at RED low register
	I2C_2_Master_RepStart();
	I2C_2_Master_Write(0x52 | 0x01);     //7 bit address + Read (1) mode
    tmp=I2C_2_Master_Read(1);
	tmp = tmp | (I2C_2_Master_Read(0)<<8);
    
	I2C_2_Master_Stop();          //Stop condition
	return tmp;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK READ GREEN FUNCTION                                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
unsigned int color_read_Green(void)
{
	unsigned int tmp;
	I2C_2_Master_Start();         //Start condition
	I2C_2_Master_Write(0x52 | 0x00);     //7 bit address + Write mode
	I2C_2_Master_Write(0xA0 | 0x18);    //command (auto-increment protocol transaction) + start at RED low register
	I2C_2_Master_RepStart();
	I2C_2_Master_Write(0x52 | 0x01);     //7 bit address + Read (1) mode
    tmp=I2C_2_Master_Read(1);
	tmp = tmp | (I2C_2_Master_Read(0)<<8);
    
	I2C_2_Master_Stop();          //Stop condition
	return tmp;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK READ BLUE FUNCTION                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
unsigned int color_read_Blue(void)
{
	unsigned int tmp;
	I2C_2_Master_Start();         //Start condition
	I2C_2_Master_Write(0x52 | 0x00);     //7 bit address + Write mode
	I2C_2_Master_Write(0xA0 | 0x1A);    //command (auto-increment protocol transaction) + start at RED low register
	I2C_2_Master_RepStart();
	I2C_2_Master_Write(0x52 | 0x01);     //7 bit address + Read (1) mode
    tmp=I2C_2_Master_Read(1);
	tmp = tmp | (I2C_2_Master_Read(0)<<8);
    
	I2C_2_Master_Stop();          //Stop condition
	return tmp;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK READ CLEAR FUNCTION                                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
unsigned int color_read_RGBC(void)
{
	unsigned int tmp;
	I2C_2_Master_Start();         //Start condition
	I2C_2_Master_Write(0x52 | 0x00);     //7 bit address + Write mode
	I2C_2_Master_Write(0xA0 | 0x14);    //command (auto-increment protocol transaction) + start at RED low register
	I2C_2_Master_RepStart();
	I2C_2_Master_Write(0x52 | 0x01);     //7 bit address + Read (1) mode
    tmp=I2C_2_Master_Read(1);
	tmp = tmp | (I2C_2_Master_Read(0)<<8);
    
	I2C_2_Master_Stop();          //Stop condition
	return tmp;
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK PRINT RGB VALUE FUNCTION                                      //
//  The value of the color read values were implemented into the sprintf      //
//  function which was required for serial communication, the value was saved //
//  into the buf pointer and sent to Realterm. This was used for clarifying   //
//  the individual card colours for use with the RGBtoHue function.           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void printRGB(char *buf, unsigned int color_read) {
    sprintf(buf, "%d\r", color_read);
    sendStringSerial4(buf);
}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  COLOR CLICK CLEAR RESET FUNCTION                                          //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void color_interrupt_clearReset(void)
{
    I2C_2_Master_Start();         //Start condition
    I2C_2_Master_Write(0x52 | 0x00);     //7 bit address (of the slave peripheral) + Write mode
    I2C_2_Master_Write(0b11100110);
    I2C_2_Master_Stop();

}

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  SENSOR INTERRUPT INITIATION FUNCTION                                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void interruptsInit(void)
{
    ///// RED
    TRISBbits.TRISB1 = 1; // sensor interrupt pin
    ANSELBbits.ANSELB1 = 0; // sensor analogue pin to 0
    INT1PPS = 0b001001; // 
    
    INTCONbits.INT1EDG = 0;
    color_interrupt_clearReset(); // clear reset function

    
    color_writetoaddr(0x04, 0);
    color_writetoaddr(0x05, 0);
    color_writetoaddr(0x06, 0b00001000); // high threshold low byte
    color_writetoaddr(0x07, 0b00000000); // high threshold high byte
    color_writetoaddr(0x0C, 0b00000100); // persistence 
    color_writetoaddr(0x00, 0b10011); 
    INTCONbits.IPEN = 1; // interrupt priority enable
    IPR0bits.INT1IP = 1; // set interrupt priority
    INTCONbits.GIE = 1;   // Enable global interrupts
    PIR0bits.INT1IF = 0;  // set external interrupt flag to 0
    PIE0bits.INT1IE = 1;  // set the external interrupt
} 