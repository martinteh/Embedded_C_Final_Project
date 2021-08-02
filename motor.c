#include <xc.h>
#include <proc/pic18f67k40.h>
#include "motor.h"
#include "i2c.h"
#include "color.h"  
/////////////////////////////
/// SETUP OF WHEEL MOTORS ///
///   intialises wheels   ///   
///   called in main.c    ///
/////////////////////////////
struct DC_motor mL; // Left wheels
struct DC_motor mR; // Right wheels 
void motorsetup(void) { // functon which initilaises motors
    // Left wheels
    mL.power=0; 						//zero power to start
    mL.direction=1; 					//set default motor direction to forward
    mL.dutyHighByte=(unsigned char *)(&PWM6DCH);	//store address of PWM duty high byte
    mL.dir_LAT=(unsigned char *)(&LATE); 		//store address of LAT
    mL.dir_pin=4; 						//pin RE4 controls direction
    mL.PWMperiod=100; 			//store PWMperiod for motor
    
    // Right wheels
    mR.power=0; 						//zero power to start
    mR.direction=1; 					//set default motor direction
    mR.dutyHighByte=(unsigned char *)(&PWM7DCH);	//store address of PWM duty high byte
    mR.dir_LAT=(unsigned char *)(&LATG); 		//store address of LAT
    mR.dir_pin=6; 						//pin G6 controls direction
    mR.PWMperiod=100; 			//store PWMperiod for motor
    
}
//////////////////////////////////////////////////
///    INITIAL AMBIENT SENSOR LIGHT VALUES     ///
///  used to standardise interrupt thresholds  ///
///        initialVals() called in main.c      ///
//////////////////////////////////////////////////
unsigned int R1; // Initial Red value
unsigned int G1; // Initial Green value
unsigned int B1; // Initial Blue value
unsigned int C1; // Initial Clear value
// Function called once in main.c when buggy is first turned on
void initialVals(unsigned int R, unsigned int G, unsigned int B, unsigned int C) {
    // set each initial values equal to the respective input values 
    R1 = R; 
    B1 = B;
    G1 = G;
    C1 = C;
}
////////////////////////////////////////////////////////
///  INITIALISE PWM AND TIMER2 FOR DC MOTOR CONTROL  ///
///   used to change the power in the wheel motors   ///
///               called in main.c                   ///
////////////////////////////////////////////////////////
void initDCmotorsPWM(int PWMperiod){
	//initialise TRIS and LAT registers for PWM
    TRISEbits.TRISE2=0; // output on RE2
    TRISCbits.TRISC7=0; // output on RC7
    TRISEbits.TRISE4=0; // output on RE4
    TRISGbits.TRISG6=0; // output on G6
    
    LATEbits.LATE2=0; // 0 output initially on RB0
    LATCbits.LATC7=0; // 0 output initially on RB2 
    LATEbits.LATE4=0; // 0 output initially on RE4
    LATGbits.LATG6=0; // 0 output initially on RG6
    
    // timer 2 configuration
    T2CONbits.CKPS=0b100; // 1:16 prescaler
    T2HLTbits.MODE=0b00000; // Free Running Mode, software gate only
    T2CLKCONbits.CS=0b0001; // Fosc/4

    // Tpwm*(Fosc/4)/prescaler - 1 = PTPER
    T2PR=99; // period reg 10kHz base period
    T2CONbits.ON=1; // turn on timer2
    
    RE2PPS=0x0A; //PWM6 on RE2
    RC7PPS=0x0B; //PMW7 on RC7

    PWM6DCH=0; //0% power
    PWM7DCH=0; //0% power
    
    PWM6CONbits.EN = 1;
    PWM7CONbits.EN = 1;
}

////////////////////////////////////////////////////////
///  SEND PWM OUTPUT TO MOTORS FROM MOTOR STRUCTURES ///
///        sets the power output of the wheels       ///
///  called in motor.c for drive and turn functions  ///
////////////////////////////////////////////////////////

void setMotorPWM(struct DC_motor m)
{
	int PWMduty; //tmp variable to store PWM duty cycle

	if (m.direction){ //if forward
		// low time increases with power
		PWMduty=m.PWMperiod - ((int)(m.power)*(m.PWMperiod))/100;
	}
	else { //if reverse
		// high time increases with power
		PWMduty=((int)(m.power)*(m.PWMperiod))/100;
	}

	*(m.dutyHighByte) = PWMduty; //set high duty cycle byte
        
	if (m.direction){ // if direction is high
		*(m.dir_LAT) = *(m.dir_LAT) | (1<<(m.dir_pin)); // set dir_pin bit in LAT to high without changing other bits
	} else {
		*(m.dir_LAT) = *(m.dir_LAT) & (~(1<<(m.dir_pin))); // set dir_pin bit in LAT to low without changing other bits
	}
}

////////////////////////////////////////////////////////////////////////
///                          STOP FUNCTION                           ///
///             stops the buggy from its current action              ///
///  called in other functions in motor.c when stopping is required  ///
////////////////////////////////////////////////////////////////////////
void stop(void)
{
    TRISDbits.TRISD4 = 0;
    while (mL.power > 0 && mR.power > 0) { // while power in wheels bigger than 0
        LATDbits.LATD4 = 1; // turn on rear buggy LEDs (brake lights)
        mL.power--; // reduce power in wheels gradually to 0
        mR.power--;
        // set PWM of motors every time motor power is decreased
        setMotorPWM(mL); 
        setMotorPWM(mR);
    }
    LATDbits.LATD4 = 0; // turn off rear buggy LEDs (brake lights))
    __delay_ms(1000); // delay 1 second to allow buggy to settle before next action
}

///////////////////////////////////////////////////////////////////////////
///                        TURN RIGHT FUNCTION                          ///
///         turns the buggy clockwise around its centre of mass         ///
///  specific turn angle is set using a __delay_ and stop() function -  ///
///  after turnRight() is called                                        ///
/// called in other functions in motor.c when turning right is required ///
///////////////////////////////////////////////////////////////////////////
void turnRight(void) {
    // set direction of wheels to turn the buggy clockwise
    mL.direction=1; // set direction of left wheels to forward
    mR.direction=0; // set direction of right wheels to reverse
    TRISHbits.TRISH0 = 0;
    while (mL.power < 80 && mR.power < 80) { // power levels set to 80% so that turning can be done more accurately than at 100%
        LATHbits.LATH0 = 1; // turn on left buggy LEDs
        mL.power++; // increase power in wheels gradually to 80
        mR.power++;
        // set PWM of motors every time motor power is increased
        setMotorPWM(mL);
        setMotorPWM(mR);
    }


}
//////////////////////////////////////////////////////////////////////////
///                        TURN LEFT FUNCTION                          ///
///         turns the buggy anticlockwise around its centre of mass    ///
///  specific turn angle is set using a __delay_ and stop() function - ///
///  after turnLeft() is called                                        ///
/// called in other functions in motor.c when turning left is required ///
//////////////////////////////////////////////////////////////////////////
void turnLeft(void) {
    // set direction of wheels to turn the buggy anticlockwise
    mL.direction=0; // set direction of left wheels to reverse
    mR.direction=1; // set direction of right wheels to forward
    TRISFbits.TRISF0 = 0;
    ANSELFbits.ANSELF0 = 0;
    while (mL.power < 80 && mR.power < 80) {  // power levels set to 80% so that turning can be done more accurately than at 100%
        LATFbits.LATF0 = 1; // turn on right buggy LEDs
        mL.power++; // increase power in wheels gradually to 80
        mR.power++;
        // set PWM of motors every time motor power is increased
        setMotorPWM(mL);
        setMotorPWM(mR);
    }
}
///////////////////////////////////////////////////////////////////////////////
///                            TURN 180 FUNCTION                            ///
///         turns the buggy anticlockwise around its centre of mass         ///
///  180 degree turn angle set by intrinsic __delay_() and stop() functions - ///
///  at the end of this function                                            ///
///    called in other functions in motor.c when turning 180 is required    ///
///////////////////////////////////////////////////////////////////////////////
void turn180 (void) {
    // set direction of wheels to turn the buggy anticlockwise
    mL.direction=0; // set direction of left wheels to reverse
    mR.direction=1; // set direction of right wheels to forward
    TRISFbits.TRISF0 = 0;
    ANSELFbits.ANSELF0 = 0;
    // power levels set to 70% so that turning can be done more accurately than 100% and 80% (as for other turn functions)
    // as buggy starts to skid at high turn speeds
    while (mL.power < 70 && mR.power < 70) {  
        LATFbits.LATF0 = 1; // turn on left buggy LEDS
        mL.power++; // increase power in wheels gradually to 70%
        mR.power++;
        // set PWM of motors every time motor power is increased
        setMotorPWM(mL);
        setMotorPWM(mR);
    }
    __delay_ms(550); // specific delay function so that buggy turns 180 on the floor that was demonstrated on
    stop(); // stop turning
}

///////////////////////////////////////////////////////////////////////////////
///                            FORWARD FUNCTION                             ///
///               propels buggy forward in a straight line                  ///
///  specific travel distance is set using a __delay_ and stop() function - ///
///  after fullSpeedAhead() is called                                       ///
///     called in other functions in motor.c and in main.c when needed      ///
///////////////////////////////////////////////////////////////////////////////
void fullSpeedAhead(void)
{    // set direction of wheels to go straight forward
    mL.direction=1; // set left wheel motors to forward
    mR.direction=1; // set right wheel motors to forward
    TRISDbits.TRISD3 = 0;
    // 18% power chosen so that forward motion is not too fast for
    // sensor values to be read and sensor interrupt to be triggered
    while (mL.power < 18 && mR.power < 18) {
        LATDbits.LATD3 = 1; // turn on white buggy headlamps
        mL.power++; // increase power in wheels gradually to 18%
        mR.power++;
        // set PWM of motors every time motor power is incremented
        setMotorPWM(mL);
        setMotorPWM(mR);
    }
}
//////////////////////////////////////////////////////////////////////////////////
///                             BACKWARD FUNCTION                              ///
///               propels buggy backwards in a straight line                   ///
///  specific travel distance is set using a __delay_ and stop() function -    ///
///  at the end of this function                                               ///
///  used to reverse from the coloured cards when they are detected so that -  ///
///  there is room for the buggy to turn and drive                             ///
///             called in other functions in motor.c when needed               ///
//////////////////////////////////////////////////////////////////////////////////
void backwards(void)
{    // set direction of wheels to go backwards
    mL.direction=0; // set direction of left wheels to reverse
    mR.direction=0; // set direction of right wheels to reverse
    TRISDbits.TRISD3 = 0;
    while (mL.power < 70 && mR.power < 70) { // set power to 70% so that reverse is small 
        mL.power++; // increase power in wheels gradually to 70%
        mR.power++;
        // set PWM of motors every time motor power is increased
        setMotorPWM(mL);
        setMotorPWM(mR);
    }
    __delay_ms(200); // intrinsic delay so that buggy reverses a set amount every time
    stop(); // stop reversing
}
///////////////////////////////////////////////////////////////////////////////////////
///                                 ACTION FUNCTION                                 ///
///        decides on the necessary action depending on the hue value inputed       ///
///      relative hue threshold values are dependent on the ambient light used      ///
///  called in SensorISR() when the light value increases over the threshold value  ///
///////////////////////////////////////////////////////////////////////////////////////
void action(unsigned int hue) {
   // LIGHT BLUE - 180 DEGREE TURN
    if (hue >= 190 && hue <= 220) {
        backwards(); // reverse backwards and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        moveArray(1); // add 1 (representing light blue) into the movement array
        turn180(); // turn 180 degrees and stop
        stop();
        LATFbits.LATF0 = 0; // turn off left buggy LEDs
        fullSpeedAhead(); // carry on forward
    
    } else if (hue >= 70 && hue <= 95) { // LIGHT GREEN - 135 DEGREE TURN LEFT
        backwards(); // reverse backward and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        turnLeft(); // turn anticlockwise
        __delay_ms(301); // specific delay for the surface tested on
        stop(); // stop turning
        LATFbits.LATF0 = 0; // turn off left buggy LEDs
        fullSpeedAhead(); // carry on forward
    } else if (hue >= 110 && hue <= 145) { // DARK GREEN - 90 DEGREE TURN LEFT
        backwards(); // reverse backward and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        moveArray(3); // add 3 (representing dark green) into the movement array
        turnLeft(); // turn anticlockwise
        __delay_ms(195); // specific delay for the surface tested on
        stop(); // stop turning
        LATFbits.LATF0 = 0; // turn off left buggy LEDs
        fullSpeedAhead();  // carry on forward
    } else if (hue >= 270 && hue <= 320) { // PURPLE - 135 DEGREE TURN RIGHT
        backwards(); // reverse backward and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        moveArray(4); // add 4 (representing purple) into the movement array
        turnRight(); // turn clockwise
        __delay_ms(300); // specific delay for the surface tested on
        stop();  // stop turning
        LATHbits.LATH0 = 0; // turn off right buggy LEDs
        fullSpeedAhead(); // carry on forward
    } else if (hue >= 325 && hue <= 352) {  // PINK - 60 DEGREE TURN RIGHT
        backwards(); // reverse backward and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        moveArray(5);  // add 5 (representing pink) into the movement array
        turnRight(); // turn clockwise
        __delay_ms(139); // specific delay for the surface tested on
        stop(); // stop turning
        LATHbits.LATH0 = 0; // turn off right buggy LEDs
        fullSpeedAhead(); // carry on forward
    } else if (hue >= 355 && hue <= 360 || hue == 0) {  // RED, ORANGE OR BLACK = ALL RETURN SIMILAR HUE VALUES SO MUST BE CHECKED FURTHER
          /* if light intensity is over a threshold of 15 then the colour is either black or red
         to distinguish between them further, the buggy drives forward a little extra and
         rereads the values */
        if (color_read_RGBC()>= 15) {
            // drive forward for 200ms
            fullSpeedAhead();
            __delay_ms(200);
            stop();
            // reread the sensor values and calculate the hue
            unsigned int R = color_read_Red();
            unsigned int G = color_read_Green();
            unsigned int B = color_read_Blue();
            unsigned int hue = RGBtoHue(R,G,B);
            // RED - 90 DEGREE TURN RIGHT
            if (hue >= 355 && hue <= 360 || hue == 0) {
            backwards(); // reverse backward and stop
            moveArray(fwrdTime); // add the forward time into the movement array
            fwrdTime = 0; // reset forward time back to 0
            moveArray(6); // add 6 (representing red) into the movement array
            turnRight(); // turn clockwise
            __delay_ms(195); // specific delay for the surface tested on
            stop(); // stop turning
            LATHbits.LATH0 = 0; // turn off right buggy LEDs
            fullSpeedAhead(); // carry on forward
            } else if (hue >= 1 && hue <= 5) { // ORANGE - TURN LEFT 30
            backwards(); // reverse backward and stop
            stop(); // add the forward time into the movement array
            moveArray(fwrdTime); // add the forward time into the movement array
            fwrdTime = 0; // reset forward time back to 0
            moveArray(7); // add 7 (representing orange) into the movement array
            turnLeft(); // turn anticlockwise
            __delay_ms(90); // specific delay for the surface tested on
            stop(); // stop turning
            LATFbits.LATF0 = 0;  // turn off left buggy LEDs
            fullSpeedAhead(); // carry on forward
            }
        } else if (color_read_RGBC()< 13 && color_read_RGBC() >= 5) {  //BLACK
        // BLACK IS REGISTERED WHEN THE CLEAR READING IS HIGHER THAN 5 BUT LOWER THAN 13 WHILST STILL OUTPUTING A HUE OF AROUND 0
        backwards(); // reverse backward and stop
        TRISHbits.TRISH0 = 0;
        // turn on left and right buggy LEDs to show its in the reverse protocol
        LATHbits.LATH0 = 1;
        LATFbits.LATF0 = 1;
        turn180(); // turn 180 degrees
        backwards(); // reverse backward and stop
        moveArray(fwrdTime - 3); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        reverse(); // activate reverse protocol
        } else { // else carry on 
        fullSpeedAhead();
}
    } else if (hue >= 18 && hue <= 35) { // YELLOW - 30 DEGREE TURN RIGHT
        backwards(); // reverse backward and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        moveArray(8); // add 8 (representing yellow) into the movement array
        turnRight();  // turn clockwise
        __delay_ms(95); // specific delay for the surface tested on
        stop();  // stop turning
        LATHbits.LATH0 = 0; // turn off right buggy LEDs
        fullSpeedAhead(); // carry on forward
    } else if (hue >= 225 && hue <= 268) { // DARK BLUE - 60 DEGREE TURN LEFT 
        backwards(); // reverse backward and stop
        moveArray(fwrdTime); // add the forward time into the movement array
        fwrdTime = 0; // reset forward time back to 0
        moveArray(9); // add 9 (representing dark blue) into the movement array
        turnLeft(); // turn anticlockwise
        __delay_ms(139); // specific delay for the surface tested on
        stop(); // stop turning
        LATFbits.LATF0 = 0; // turn off left buggy LEDs
        fullSpeedAhead(); // carry on forward
    } else { // else carry on if no specific values are detected 
        fullSpeedAhead();
}
}
////////////////////////////////////////////////////////////////////////////////
///            SENSOR INTERRUPT SERVICE ROUTINE - HIGH PRIORITY              ///
/// when threshold is reached, interrupt flag INT1IF is triggered and this - ///
/// routine is carried out                                                   ///
///                   interrupt is initialised in color.c                    ///
////////////////////////////////////////////////////////////////////////////////
void __interrupt(high_priority) SensorISR()
{
    if (PIR0bits.INT1IF && PIE0bits.INT1IE == 1) // if sensor/colour click interrupt interrupt flag is triggered
    {
        stop(); // stop the buggy
        // read the RGB sensor values and convert to hue 
        unsigned int hue = RGBtoHue(color_read_Blue(),color_read_Green(),color_read_Red());
        action(hue); // carry out an action dependent on the hue value
        color_interrupt_clearReset(); // reset the colour click interrupt      
        PIR0bits.INT1IF = 0; // set the interrupt flag back low
    }
}

////////////////////////////////////////////////////////////////////////////////
///            TIMER2 INTERRUPT SERVICE ROUTINE - LOW PRIORITY               ///
///     timer resets and triggers the interrupt flag after a set time        ///
///    timer overflow threshold is set to 62.5ms in the initialisation       ///
///                   interrupt is initialised in i2c.c                      ///
////////////////////////////////////////////////////////////////////////////////
// fwrdtime (forward time) tracks the time that the buggy travels forward before an action is commenced
int fwrdTime = 0;
void __interrupt(low_priority) TimerISR()
{
    if (PIR0bits.TMR0IF) // if flag is set on timer overflow
    {
        fwrdTime++; // increment fwrdTime
        LATDbits.LATD7 = !LATDbits.LATD7; //flip clicker LED to show timer running
        if (fwrdTime == 160) { // if fwrdTime reaches 160 (equivalent to 10 seconds), the reverse protocol is triggered
            // disable interrupts
            INTCONbits.GIE=0;
            PIE0bits.INT1IE = 0;
            PIE0bits.TMR0IE = 0;
            stop(); // stop buggy
            TRISHbits.TRISH0 = 0;
            LATHbits.LATH0 = 1;
            LATFbits.LATF0 = 1;
            backwards(); // reverse backwards and stop 
            turn180(); // turn 180 degrees
            backwards(); // turn 180 and stop
            moveArray(fwrdTime); // add the forward time into the movement array
            reverse(); // activate reverse protocol
        }
    }
    PIR0bits.TMR0IF=0; // set interrupt flag back to low
    
    
}
////////////////////////////////////////////////////////////////////////////////////////
///                                   MOVE ARRAY                                     ///
///                    adds the value n into the movement array                      ///
///  stores the actions performed by the buggy so that they can be reversed in the - ///
///  reverse protocol                                                                ///
////////////////////////////////////////////////////////////////////////////////////////
int i=0; // index i tracks the current position that is occupied in the movement array
/* MOVEMENT ARRAY IS AN ARRAY OF LENGTH 21 THAT TRACKS IN THIS ORDER:
 [TIME,ACTION,TIME,ACTION,......,ACTION,TIME] WHERE TIME IS THE TIME
 TRAVELLED FORWARD AND ACTION IS THE NUMBER REFERRING TO THE TURNING ACTION
*/
// 21 is chosen as the length so that it fills up when it has read 10 cards 
int movement[20] = {0};
void moveArray(int n){
// if the array is not filled up, increment the index
    if (i<21){ 
        i++;
    }
    // if after the increment, the array is not filled, then add the value to the array
    if (i<=20){
        movement[i-1] = n;
    }
    else { // else turn 180 and activate the reverse protocol
        backwards(); // reverse backwards and stop 
        turn180(); // turn 180 degrees
        backwards(); // turn 180 and stop
        reverse(); // activate reverse protocol
    }
}
//////////////////////////////////////////////////////////////////////
///                      REVERSE PROTOCOL                          ///
/// this protocol turns retraces the path that the buggy has taken ///
//////////////////////////////////////////////////////////////////////
void reverse(void){
    // disable interrupts
    INTCONbits.GIE = 0;
    PIE0bits.INT1IE = 0;
    PIE0bits.TMR0IE = 0;
    // starting from the end of the array, the algorithm works down in decrements of 2
    // alternating between forward and action
    int l = 20;
        while (l>0){
            reverseFwrd(movement[l]);
            reverseAction(movement[l-1]);

        
        l-=2;
}
    reverseFwrd(movement[0]);
    Sleep(); // send buggy to sleep
}
///////////////////////////////////////
///         REVERSE FORWARD         ///
/// retraces the time going forward ///
///////////////////////////////////////
void reverseFwrd(int t) { // input of fwrdTime (t)
    if (t != 0){ // if time is not zero
        int j = 0; // increment j until it reaches t
    // travel forward in delays of 62.5ms (how often fwrdTime increments)
    fullSpeedAhead(); 
    while (j<t){
    __delay_ms(62);
    __delay_us(500);
    j++;
    }
    j = 0; // reset j so it can be used another time
    stop(); // stop once j has equalled t
    }
    
}
/////////////////////////////////////////////////////////////
///         REVERSE Action                                ///
/// retraces the actions carried out                      ///
/// carries out the opposite actions to those in action() ///
/////////////////////////////////////////////////////////////
void reverseAction(int n){
    if (n!=0) {
      if (n==1) { //light blue
        // turn 180
        turn180();
        stop();

        backwards();
    } else if (n==2) { //light green
        // turn right 135
        turnRight();
        __delay_ms(300);
        stop();
        backwards();
    } else if (n==3) { //dark green
        //turn right 90
        turnRight();
        __delay_ms(195);
        stop();
        backwards();
    } else if (n==4) { //purple
        //turn left 135
        turnLeft();
        __delay_ms(300);
        stop();
        backwards();
    } else if (n==5) { //pink
        // turn left 60
        stop();
        turnLeft();
        __delay_ms(135);
        stop();
        backwards();
    } else if (n==6) { //red
        // turn left 90
        turnLeft();
        __delay_ms(195);
        stop();
        backwards();
        __delay_ms(1000);
    } else if (n==7) { //orange
        // turn right 30
        turnRight();
        __delay_ms(90);
        stop();
        backwards();
    } else if (n==8) { //yellow
        // turn left 30
        turnLeft();
        __delay_ms(95);
        stop();
        backwards();

    } else if (n==9) { //dark blue

        //turn right 60
        turnRight();
        __delay_ms(135);
        stop();
        backwards();
}
    __delay_ms(1000); // delay between actions
    }
}
    
