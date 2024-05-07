
// PIC16F877A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


#define _XTAL_FREQ 20000000
#define TMR2PRESCALE 4

// ??nh ngh?a các chân ?i?u khi?n ??ng c?
#define IN1 RD2
#define IN2 RD3
#define IN3 RC4
#define IN4 RC5
// Khai báo bi?n toàn c?c 
int error = 0;
// PID 

// Thay ??i và tìm giá tr? phù h?p ?? xe ???c ?n ??nh h?n 
float Kp = 100; 
              
float Ki = 0.0008;
              
float Kd = 0.6; 
              
int P;
int I;
int D;
int lastError = 0;

// Toc do dong co khoang: 0 - 1023 
const int maxspeeda = 1000; 
const int maxspeedb = 1000;
const int basespeeda = 800;
const int basespeedb = 800;





long freq;

int PWM_Max_Duty() {
    return (_XTAL_FREQ / (freq * TMR2PRESCALE));
}

void PWM1_Init(long fre) {
    PR2 = (_XTAL_FREQ / (fre * 4 * TMR2PRESCALE)) - 1;
    freq = fre; // S?a l?i gán giá tr?
}

void PWM2_Init(long fre) {
    PR2 = (_XTAL_FREQ / (fre * 4 * TMR2PRESCALE)) - 1;
    freq = fre;
}

void PWM1_Duty(unsigned int duty) {
    if (duty < 1024) {
        duty = ((float) duty / 1023) * PWM_Max_Duty();
        CCP1X = (duty & 2) >> 1;
        CCP1Y = duty & 1;
        CCPR1L = duty >> 2;
    }
}

void PWM2_Duty(unsigned int duty) {
    if (duty < 1024) {
        duty = ((float) duty / 1023) * PWM_Max_Duty();
        CCP2X = (duty & 2) >> 1;
        CCP2Y = duty & 1;
        CCPR2L = duty >> 2;
    }
}

void PWM1_Start() {
    CCP1M3 = 1;
    CCP1M2 = 1;
    T2CKPS0 = 1;
    T2CKPS1 = 0;
    TMR2ON = 1;
    TRISC2 = 0;
}

void PWM2_Start() {
    CCP2M3 = 1;
    CCP2M2 = 1;
    T2CKPS0 = 1;
    T2CKPS1 = 0;
    TMR2ON = 1;
    TRISC1 = 0;
}

void PWM1_Stop() {
    CCP1M3 = 0;
    CCP1M2 = 0;
}

void PWM2_Stop() {
    CCP2M3 = 0;
    CCP2M2 = 0;
}

void delay_ms(unsigned int delay_time) {
    while (delay_time--) {
        __delay_ms(1);
    }
}

void move_forward(float speedA, float speedB) {
    PWM1_Duty(speedA);
    PWM2_Duty(speedB);
    IN1 = 0;
    IN2 = 1;
    IN3 = 1;
    IN4 = 0;
}

void move_A(float speedA, float speedB) {
    PWM1_Duty(speedA);
    PWM2_Duty(speedB);
    IN1 = 0;
    IN2 = 1;
    IN3 = 0;
    IN4 = 1;
}
void move_B(float speedA, float speedB) {
    PWM1_Duty(speedA);
    PWM2_Duty(speedB);
    IN1 = 1;
    IN2 = 0;
    IN3 = 1;
    IN4 = 0;
}


void move_backward(float speedA, float speedB) {
    PWM1_Duty(speedA);
    PWM2_Duty(speedB);
    IN1 = 1;
    IN2 = 0;
    IN3 = 0;
    IN4 = 1;
}


void main() {
    
    // Tan so dao dong 
    PWM1_Init(5000);
    PWM2_Init(5000);
    
    // ??t PORTD là OUTPUT
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;// ??t toàn b? PORTD là ??u ra
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;  
    
    TRISEbits.TRISE0 = 0;
    
       // Thi?t l?p chân c?m bi?n input 
    TRISBbits.TRISB0 = 1; 
    // T?t ng?t ngo?i vi n?u không c?n
    INTCONbits.GIE = 0; // T?t Global Interrupts n?u không c?n
    OPTION_REGbits.nRBPU = 0; // Kích ho?t weak pull-up trên PORTB
    
    TRISBbits.TRISB1 = 1; 
    TRISBbits.TRISB2 = 1; 
    TRISBbits.TRISB3 = 1; 
    TRISBbits.TRISB4 = 1; 
    
    TRISDbits.TRISD6 = 1; 
    TRISDbits.TRISD5 = 1; 
    TRISDbits.TRISD4 = 1; 
    TRISCbits.TRISC7 = 1; 
    
   
    PWM1_Duty(0);
    PWM2_Duty(0);
    PWM1_Start();
    PWM2_Start();
    
    PORTEbits.RE0 = 1;
    while(1){
        int L1 = PORTCbits.RC7;
        int L2 = PORTDbits.RD4;
        int L3 = PORTDbits.RD5;
        int L4 = PORTDbits.RD6;
        
        int L5 = PORTBbits.RB2;
        int L6 = PORTBbits.RB3;
        int L7 = PORTBbits.RB4;
        int L8 = PORTBbits.RB1;
        // Chu y:" Gia tri 1 nen TRANG , gia tri 0 nen DEN"
        int TRANG = 1;
        int DEN = 0;
        
        if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == DEN && L5 == DEN && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = 0;
            //move_forward(500, 500);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == DEN && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = 1;
            //move_forward(0, 500);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == DEN && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -1;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == DEN && L7 == TRANG && L8 == TRANG){
            error = 2;
            //move_forward(0, 500);
        }else if(L1 == TRANG && L2 == TRANG && L3 == DEN && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -2;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == DEN && L8 == TRANG){
            error = 50;
            //move_forward(0, 500);
        }else if(L1 == TRANG && L2 == DEN && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -50;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == DEN){
            error = 100;
            //move_forward(0, 500);
        }else if(L1 == DEN && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -100;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == DEN && L8 == DEN){
            error = 200;
            //move_forward(0, 500);
        }else if(L1 == DEN && L2 == DEN && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -200;
            //move_forward(500, 0);
        }else if(L6 == DEN && L7 == DEN && L8 == DEN){
            error = 300;
            //move_forward(0, 500);
        }else if(L1 == DEN && L2 == DEN && L3 == DEN ){
            error = -300;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == DEN && L6 == DEN && L7 == DEN && L8 == DEN){
            error = 400;
            //move_forward(0, 500);
        }else if(L1 == DEN && L2 == DEN && L3 == DEN && L4 == DEN && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -400;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == DEN && L5 == DEN && L6 == DEN && L7 == DEN && L8 == DEN){
            error = 500;
            //move_forward(0, 500);
        }else if(L1 == DEN && L2 == DEN && L3 == DEN && L4 == DEN && L5 == DEN && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -500;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == DEN && L5 == DEN && L6 == DEN && L7 == DEN && L8 == TRANG){
            error = 600;
            //move_forward(0, 500);
        }else if(L1 == TRANG && L2 == DEN && L3 == DEN && L4 == DEN && L5 == DEN && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -600;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == DEN && L5 == DEN && L6 == DEN && L7 == TRANG && L8 == TRANG){
            error = 700;
            //move_forward(0, 500);
        }else if(L1 == TRANG && L2 == TRANG && L3 == DEN && L4 == DEN && L5 == DEN && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = -700;
            //move_forward(500, 0);
        }else if(L1 == TRANG && L2 == TRANG && L3 == TRANG && L4 == TRANG && L5 == TRANG && L6 == TRANG && L7 == TRANG && L8 == TRANG){
            error = 4000;
            //move_forward(500, 0);
        }

        if( error == 50 ){
            move_A(800, 800);
        }else if(error == -50){
            move_B(800, 800);
        }else if( error == 100 || error == 200 || error == 300 || error == 400 || error == 500 || error == 600 || error == 700){
            move_A(1000, 1000);
            __delay_ms(50);
        }else if( error == -100 || error == -200 || error == -300 || error == -400 || error == -500 || error == -600 || error == -700){
            move_B(1000, 1000);
            __delay_ms(50);
            
        }else{
        P = error;
        I = I + error;
        D = error - lastError;
        lastError = error;
        int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  
        int motorspeeda = basespeeda + motorspeed;
        int motorspeedb = basespeedb - motorspeed;
  
        if (motorspeeda > maxspeeda) {
            motorspeeda = maxspeeda;
        }
        if (motorspeedb > maxspeedb) {
            motorspeedb = maxspeedb;
        }
        if (motorspeeda < 30) {
            motorspeeda = 0;
        }
        if (motorspeedb < 30) {
            motorspeedb = 0;
        } 

        // DK toc do dong co 
        move_forward(motorspeeda, motorspeedb);
        
        }
    }
}
    

       
