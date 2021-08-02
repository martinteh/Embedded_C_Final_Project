#ifndef _MOTOR_H
#define _MOTOR_H

#include <xc.h>

#define _XTAL_FREQ 64000000

struct DC_motor { //definition of DC_motor structure
    char power;         //motor power, out of 100
    char direction;     //motor direction, forward(1), reverse(0)
    unsigned char *dutyHighByte; //PWM duty high byte address
    unsigned char *dir_LAT; //LAT for dir pin
    char dir_pin; // pin number that controls direction on LAT
    int PWMperiod; //base period of PWM cycle
};
void motorsetup(void);
//function prototypes
void initDCmotorsPWM(int PWMperiod); // function to setup PWM
void setMotorPWM(struct DC_motor m);
void stop(void);
void turnLeft(void);
void turnRight(void);
void fullSpeedAhead(void);
void __interrupt(high_priority) HighISR();
void action(unsigned int hue);
void moveArray(int n);
void __interrupt(low_priority) LowISR();
void reverse(void);
void reverseFwrd(int t);
void reverseAction(int n);
void turn180 (void);
int fwrdTime;
#endif
