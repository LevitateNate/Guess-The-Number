/*===================================CPEG222====================================
 * Program:      Project 3 template
 * Authors:     Nathan Ekanem & Devereau Zeleznik
 * Date:        10/19/2023
 * This is a template that you can use to write your project 3 code, for mid-stage and final demo.
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/

#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "lcd.h"    // Digilent Library for using the on-board LCD
#include "acl.h"    // Digilent Library for using the on-board accelerometer

#define TRUE 1
#define FALSE 0

// below are keypad row and column definitions based on the assumption that JB will be used and columns are CN pins
// If you want to use JA or use rows as CN pins, modify this part
#define R4 LATCbits.LATC14
#define R3 LATDbits.LATD0
#define R2 LATDbits.LATD1
#define R1 LATCbits.LATC13
#define C4 PORTDbits.RD9
#define C3 PORTDbits.RD11
#define C2 PORTDbits.RD10
#define C1 PORTDbits.RD8

typedef enum _KEY {K0, K1, K2, K3, K4, K5, K6, K7, K8, K9, K_A, K_B, K_C, K_D, K_E, K_F, K_NONE} eKey ;
typedef enum _MODE {MODE1,MODE2,MODE3,MODE4} eModes ;

eModes mode = MODE1;
int digit1;
int digit2;
int enter_digit;
int Win = 0;
int x = 0;
int array[4];
int rangelow[2] = {0,0};
int low = 0;
int rangehi[2] = {9,9};
int hi = 99;
int player;
int timer_on;
int timer;
int randomnum = 0;
char new_press = FALSE;

// subrountines
void SSD_WriteDigits();
void LED_Init();
void SSD_Init();
void CNConfig();
void handle_new_keypad_press(eKey key) ;
int randomnumber();

int main(void) {
    SSD_WriteDigits(31,31,31, 31, 0, 0, 0, 0);
    /* Initialization of LED, LCD, SSD, etc */
    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO
    LCD_Init();
    ACL_Init();
    SSD_Init();

    float rgACLGVals[3];
    ACL_ReadGValues(rgACLGVals);
    int seed = rgACLGVals[0] * 10000;
    srand((unsigned) seed);
    // below are keypad row and column configurations based on the assumption that JB will be used and columns are CN pins
    // If you want to use JA or use rows as CN pins, modify this part

    // keypad rows as outputs
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    ANSELDbits.ANSD1 = 0;
    TRISCbits.TRISC14 = 0;
    TRISCbits.TRISC13 = 0;

    // keypad columns as inputs
    TRISDbits.TRISD8 = 1;
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD10 = 1;
    TRISDbits.TRISD11 = 1;
    
    // You need to enable all the rows
    R1 = R2 = R3 = R4 = 0;
    
        LCD_WriteStringAtPos("Number Guessing",0,0);
        LCD_WriteStringAtPos("# players? [1-4]",1,0);
    CNConfig();
    LED_Init();
    LATA &= 0xFF00;
    /* Other initialization and configuration code */

    while (TRUE) 
    {
        //You can put key-pad independent mode transition here, such as countdown-driven mode transition, monitoring of microphone or update of RGB.
    }
} 


void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
    macro_disable_interrupts;
    
    //Complete the following configuration of CN interrupts, then uncomment them
    CNCONDbits.ON = 1;   //all port D pins to trigger CN interrupts
    CNEND = 0x0f00;      	//configure PORTD pins 8-11 as CN pins
    CNPUD = 0x0f00;      	//enable pullups on PORTD pins 8-11

    IPC8bits.CNIP = 5;  	// set CN priority to  5
    IPC8bits.CNIS = 3;  	// set CN sub-priority to 3

    IFS1bits.CNDIF = 0;   	//Clear interrupt flag status bit
    IEC1bits.CNDIE = 1;    	//Enable CN interrupt on port D
    
    
    int j = PORTD;             //read port to clear mismatch on CN pins
    macro_enable_interrupts();	// re-enable interrupts
}


void __ISR(_CHANGE_NOTICE_VECTOR) CN_Handler(void) {
    eKey key = K_NONE;
    int num = 0;
    
    
    // 1. Disable CN interrupts
    IEC1bits.CNDIE = 0;     

    // 2. Debounce keys for 10ms
    for (int i=0; i<1426; i++) {}

    // 3. Handle "button locking" logic

    unsigned char key_is_pressed = (!C1 || !C2 || !C3 || !C4);
    // If a key is already pressed, don't execute the rest second time to eliminate double pressing
    if (!key_is_pressed)
    {
        new_press = FALSE;
    }
    else if (!new_press)
    {
        new_press = TRUE;

        // 4. Decode which key was pressed
        
        
        // check first row 
        R1 = 0; R2 = R3 = R4 = 1;
        if (C1 == 0) { K1; key = 1; }      // first column
        else if (C2 == 0) { K2; key = 2; } // second column
        else if (C3 == 0) {  K3; key = 3; } // third column
        else if (C4 == 0) { K_A; key = 10; } // fourth column

        // check second row 
        R2 = 0; R1 = R3 = R4 = 1;
        if (C1 == 0) { K4; key = 4; }
        else if (C2 == 0) { K5; key = 5; }
        else if (C3 == 0) { K6; key = 6; }
        else if (C4 == 0) { K_B; key = 11; }

        // check third row 
        R3 = 0; R2 = R1 = R4 = 1;
        if (C1 == 0) {  K7; key = 7;}
        else if (C2 == 0) {  K8; key = 8;}
        else if (C3 == 0) {  K9; key = 9;}
        else if (C4 == 0) { K_C; key = 12;}

        // check fourth row 
        R4 = 0; R2 = R1 = R3 = 1;
        if (C1 == 0) {  K0; key = 0;}
        else if (C2 == 0) {  K_F; key = 15;}
        else if (C3 == 0) {  K_E; key = 14;}
        else if (C4 == 0) { K_D; key = 13;}
        

        // re-enable all the rows for the next round
        R1 = R2 = R3 = R4 = 0;
        }
    
    

        handle_new_keypad_press(key) ;
    
    
    int j = PORTD;              //read port to clear mismatch on CN pints
    
    // 5. Clear the interrupt flag
    IFS1bits.CNDIF = 0;     

    // 6. Reenable CN interrupts
    IEC1bits.CNDIE = 1; 

}

void Timer2()
{
  PR2 = (int) (((float) (10000000) / 256) + 0.5); //one interrupt every 10 ms   //set period register, generates one interrupt every 3 ms
  TMR2 = 0;                           //    initialize count to 0
  T2CONbits.TCKPS = 2;                //    1:64 prescale value
  T2CONbits.TGATE = 0;                //    not gated input (the default)
  T2CONbits.TCS = 0;                  //    PCBLK input (the default)
  T2CONbits.ON = 1;                   //    turn on Timer2
  IPC2bits.T2IP = 6;                  //    priority
  IPC2bits.T2IS = 3;                  //    subpriority
  IFS0bits.T2IF = 0;                  //    clear interrupt flag
  IEC0bits.T2IE = 1;                  //    enable interrupt
  macro_enable_interrupts();          //    enable interrupts at CPU
}

void __ISR(_TIMER_2_VECTOR) Timer2ISR(void){
    IEC0bits.T2IE = 0;
    if(timer_on == TRUE){
        if(timer > 0 ){
        timer --;
            if (timer % 100 == 0){
               SSD_WriteDigits(31,31,31, 31, 0, 0, 0, 0);
    }else{
            SSD_WriteDigits(31,31,31, 31, 0, 0, 0, 0);
            digit1 = -1;
            digit2 = -1;
            enter_digit = 0;
    }
    IFS0bits.T2IF = 0; // Clear Interrupt
    IEC0bits.T2IE = 1; // Enable Interrupt
    }
    }
}

int randomnumber(){
    randomnum = (rand()%100);
    return randomnum;
}

void handle_new_keypad_press(eKey key)
{
    switch (mode)
    {
    case MODE1:
        LCD_WriteStringAtPos("Number Guessing",0,0);
        LCD_WriteStringAtPos("# players? [1-4]",1,0);
        SSD_WriteDigits(31,31,31, 31, 0, 0, 0, 0);
        
        if (key == 1){
            LCD_WriteStringAtPos("Num Guessing -1     ",0,0);
            player = 1;
            mode = MODE2;
        }
        if (key == 2){
            LCD_WriteStringAtPos("Num Guessing -2     ",0,0);
            player = 2;
            mode = MODE2;
        }
        if (key == 3){
            LCD_WriteStringAtPos("Num Guessing -3     ",0,0);
            player = 3;
            mode = MODE2;
        }
        
        if (key == 4){
            LCD_WriteStringAtPos("Num Guessing -4     ",0,0);
            player = 4;
            mode = MODE2;
        }
        
    break;
    
    case MODE2: //generates random number
        SSD_WriteDigits(31,31,31,31,0,0,0,0);
        LCD_WriteStringAtPos("Rand(E), Det(D)   ",1,0);
        if (key == 13){
            Win = 88;
            LCD_WriteStringAtPos("Deterministic     ",1,0);
            mode = MODE3;
        }
        else if (key == 14) {
            LCD_WriteStringAtPos("Random secret    ",1,0);
            Win = randomnumber();
            mode = MODE3;
        }
    break;
    
    
    case MODE3: //guess
        timer = 0;
        if(player  == 1){
            LATA |= 0xFF01; //leds 0-3 on (i think)
        }
        if(player  == 2){
            LATA |= 0xFF02; //leds 0-3 on (i think)
        }
        if(player  == 3){
            LATA |= 0xFF04; //leds 0-3 on (i think)
        }
        if(player  == 4){
            LATA |= 0xFF08; //leds 0-3 on (i think)
        }
        
        
    break;
    
    
    case MODE4: //displays range
        SSD_WriteDigits(rangehi[1],rangehi[0],rangelow[1],rangelow[0],0,0,0,0);
        if(key == 15) {
            mode = MODE3;
        }
    break;
    }
}

/*
    array[x] = key;
        if(x==0){
            SSD_WriteDigits(31,31,31, array[x], 0, 0, 0, 0);
        }
        if(x==1){
            SSD_WriteDigits(31,31,array[x], array[x-1], 0, 0, 0, 0);
        }
        if(x==2){
            SSD_WriteDigits(31,array[x],array[x-1], array[x-2], 0, 0, 0, 0);
        }
        if(x>=3){
            SSD_WriteDigits(array[x],array[x-1],array[x-2], array[x-3], 0, 0, 0, 0);
        }
        x++;
 */