// CONFIG1L
#pragma config FEXTOSC = HS     // External Oscillator mode Selection bits (HS (crystal oscillator) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = EXTOSC_4PLL// Power-up default value for COSC bits (EXTOSC with 4x PLL, with EXTOSC operating per FEXTOSC bits)

// CONFIG3L
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep)

#define _XTAL_FREQ 64000000 // note intrinsic _delay function is 62.5ns at 64,000,000Hz  

#include <xc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor.h"
#include "i2c.h"
#include "color.h"
#include "LCD.h"
#include "serial.h"

void main(void) {
    color_click_init();
    //initUSART4(); // serial communication initiation
    
    initDCmotorsPWM(100); // initialise DC motor PWM
    
    char *bufR[16]; // storage for RGB values to be sent into the printRGB function 
    char *bufG[16]; // use with sprintf
    char *bufB[16];
    char *bufC[16];
    char *bufD[16];
    char *bufE[16];
    // pin initialisations
    TRISGbits.TRISG0 = 0;
    LATGbits.LATG0 = 0;
    
    TRISBbits.TRISB1 = 0;
    
    TRISDbits.TRISD3 = 0;
    LATDbits.LATD3 = 1;
    
    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;
    
    TRISHbits.TRISH3 = 0;
    LATHbits.LATH3 = 0;

    motorsetup(); // motor setup
    interruptsInit(); // initialise the sensor interrupt
    LED_interrupt_init(); // initialise the timer interrupt
    fullSpeedAhead(); // set the buggy going forward on flick of switch
    
    int x = 0;
    
    while (1) {
        unsigned int R1;
        unsigned int B1;
        unsigned int G1;
        unsigned int C1;
        if (x == 0){
    R1 = color_read_Red();
    G1 = color_read_Green();
    B1 = color_read_Blue();
    C1 = color_read_RGBC();
    color_writetoaddr(0x06, 0b00001000+C1); 
    initialVals(R1,G1,B1,C1);
    x = 1;
        } 
        // SERIAL COMMUNICATION
        /*int z;
        if (z == 31){
            unsigned int R = color_read_Red();
            unsigned int G = color_read_Green();
            unsigned int B = color_read_Blue();
            unsigned int C = color_read_RGBC();
            unsigned int hue = RGBtoHue(R,G,B);
            action(hue, &mL, &mR, C);
            color_interrupt_clearReset();

            PIE0bits.INT1IE = 1;
            
        }*/
    }
    
    }
