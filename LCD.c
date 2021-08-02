#include <xc.h>
#include "LCD.h"
#include <math.h>
#include <stdio.h>
//#include "ADC.h"
/************************************
 * Function to toggle LCD enable bit on then off
 * when this function is called the LCD screen reads the data lines
************************************/
void LCD_E_TOG(void)
{
    LATCbits.LATC2 = 1; //turn the LCD enable bit on
	__delay_us(2); //wait a short delay
    LATCbits.LATC2 = 0; //turn the LCD enable bit off again
}

/************************************
 * Function to set the 4-bit data line levels for the LCD
************************************/
void LCD_sendnibble(unsigned char number)
{
	//set the data lines here (think back to LED array output)
    if (number & 0b0001) { // bitwise AND operator for input number
        LATBbits.LATB3 = 1; // outputs data on RB3
    } else {LATBbits.LATB3 = 0;} 
    
    if (number & 0b0010) {
        LATBbits.LATB2 = 1; // outputs data on RB2
    } else {LATBbits.LATB2 = 0;}
    
    if (number & 0b0100) {
        LATEbits.LATE3 = 1; // outputs data on RE3
    } else {LATEbits.LATE3 = 0;}
    
    if (number & 0b1000) {
        LATEbits.LATE1 = 1; // outputs data on RE1
    } else {LATEbits.LATE1 = 0;}
    
    LCD_E_TOG();		//toggle the enable bit to instruct the LCD to read the data lines
    __delay_us(5);      //Delay 5uS
}


/************************************
 * Function to send full 8-bit commands/data over the 4-bit interface
 * high nibble (4 most significant bits) are sent first, then low nibble sent
************************************/
void LCD_sendbyte(unsigned char Byte, char type)
{
    // set RS pin whether it is a Command (0) or Data/Char (1) using type argument
    // send high bits of Byte using LCDout function
    // send low bits of Byte using LCDout function
    
    if (type == 0) { // if command
        LATCbits.LATC6 = 0; // set the RS pin low
    }
    else { // otherwise if data/char
        LATCbits.LATC6 = 1; // set the RS pin high
    }
    LCD_sendnibble(Byte >> 4); // high bits of Byte
    LCD_sendnibble(Byte); // low bits of Byte
    __delay_us(50);               //delay 50uS (minimum for command to execute)
}

/************************************
 * Function to initialise the LCD after power on
************************************/
void LCD_Init(void)
{

    //Define LCD Pins as Outputs and
    //set all pins low (might be random values on start up, fixes lots of issues)
    TRISBbits.TRISB3 = 0; // set RB3 as output, low (0)
    TRISBbits.TRISB2 = 0; // set RB2 as output, low (0)
    TRISEbits.TRISE3 = 0; // set RE3 as output, low (0)
    TRISEbits.TRISE1 = 0; // set RE1 as output, low (0)
    TRISCbits.TRISC2 = 0; // set RC2 as output, low (0)
    TRISCbits.TRISC6 = 0; // set RC6 as output, low (0)
    
    LATBbits.LATB3 = 0; // set RB3 as low
    LATBbits.LATB2 = 0; // set RB2 as low
    LATEbits.LATE3 = 0; // set RE3 as low
    LATEbits.LATE1 = 0; // set RE1 as low
    LATCbits.LATC2 = 0; // set RC2 as low
    LATCbits.LATC6 = 0; // set RC6 as low
    
    //Initialisation sequence code
	// follow the sequence in the GitHub Readme picture for 4-bit interface.
	// first Function set should be sent with LCD_sendnibble (the LCD is in 8 bit mode at start up)
	// after this use LCD_sendbyte to operate in 4 bit mode
    __delay_ms(50);
    LCD_sendnibble(0b0011); // function set, wait for more than 40ms after Vdd rises to 4.5V
    __delay_us(40);
    LCD_sendbyte(0b00101000,0); // function set, wait for more than 39us
    __delay_us(40);
    LCD_sendbyte(0b00101000,0); // function set, wait for more than 39us
    __delay_us(40);
    LCD_sendbyte(0b00001000,0); // display ON/OFF control
    __delay_us(40);
    LCD_sendbyte(0b00000001,0); // display clear
    __delay_ms(2);
    LCD_sendbyte(0b00000110,0); // entry mode set
 
	
    LCD_sendbyte(0b00001100,0); // to turn the LCD display back on at the end of the initialisation (not in the data sheet)
}

/************************************
 * Function to set the cursor to beginning of line 1 or 2
************************************/
void LCD_setline (char line)
{
    if (line == 1){ //Send 0x80 to set line to 1 (0x00 ddram address)
        LCD_sendbyte(0x80,0);
    }
    if (line == 2){ //Send 0xC0 to set line to 2 (0x40 ddram address)
        LCD_sendbyte(0xC0,0);
    }
}

/************************************
 * Function to send string to LCD screen
************************************/
void LCD_sendstring(char *string)
{
    while (*string != 0) { //code to send a string to LCD using pointers and LCD_sendbyte function
        LCD_sendbyte(*string++, 1);
    }
}

/************************************
 * Function to send string to LCD screen
************************************/
void LCD_scroll(void)
{
	//code here to scroll the text on the LCD screen
    LCD_sendbyte(0b00011000, 0);
}

/************************************
 * Function takes a ADC value and works out the voltage to 2 dp
 * the result is stored in buf as ascii text ready for display on LCD
 * Note result is stored in a buffer using pointers, it is not sent to the LCD
************************************/
void ADC2String(char *buf, unsigned int ADC_val){
	//code to calculate the inegeter and fractions part of a ADC value
	// and format as a string using sprintf (see GitHub readme)
    int int_part; // create int_part variable
    int_part = floor(ADC_val *(3.3/255)); // integer part of the ADC voltage value
    int frac_part; // create frac_part variable
    frac_part = floor((ADC_val * 100)*(3.3/255)) - (int_part * 100); // fractional part of the ADC voltage value
    sprintf(buf, "%d.%dV\r", int_part, frac_part); // stores the ADC value in int and frac parts together in buf pointer
}
/////////////////////////////////////////////////////////
/* LCD_CC inputs the character array bits, the row that
 the user wants the character (1 or 2) to be displayed in, 
 the column (1 to 16) to be displayed in and the 
 character number (the same number that is at the end of the
 * array name i.e. character4[])
 */ //////////////////////////////////////////////////////
void LCD_CC(int character[], int row , int col, int characNum) {
    int i; // declare i variable for the for loop
    
    if (row == 2){ // if second row, set the ddram LCD position based on column
        LCD_sendbyte(0b10101000 + col - 1, 0);
    } else {    // if first row, set the ddram LCD position based on column
        LCD_sendbyte(0b10000000 + col - 1, 0);
    }
    LCD_sendbyte(0b00000000 + characNum - 1, 1); // set the character ROM pattern based on custom character number
    for (i = 0; i < 8; i++){
        LCD_sendbyte(64+(8*(characNum-1))+i,0);   // set the CGRAM address
        LCD_sendbyte(character[i],1);   // send the character to the screen
    }   
}


