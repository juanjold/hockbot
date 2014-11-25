/* Name: robockey driver 1.0
 * Authors: Cruz, Teddy, Juanjo, & Spencer
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "m_general.h"
#include "m_wii.h"

#define motor1 PORTB
#define direction1 7 //high is forward and low is backwards
#define PWM1 6 //this is the PIN on register that will be used to PWM the motor enable... output of timer1

#define motor2 PORTC
#define direction2 7 //high is forward and low is backwards
#define PWM2 6

#define enablePIN PORTB
#define enableNUM 0 //B0 for enable on the motor driver. enables both motors

//declare variables
bool test = true;
unsigned int data[12]; // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; for storing incoming wii data
bool read_success; //determining whether or not the call to the wii was successfull or nah

int main(void)
{
    //prototype the setup functions
    void init();
    void timer1setup_cvargas(int time_scale);
    void timer3setup_cvargas(); //defaults to timescale of 0
    void wii_setup(){};
    
    //prototype functions to be used in main loop
    void go(char direction);
    void drivetest();
    
    //prototype sub functions (not to be used in main)
    void timerswitch(bool on);
    
    //prototype functions that I'm not sure where they'll go yet
    void wii_read(){};
    
    //call the external setup functions
    m_red(ON);
    timer1setup_cvargas(0);
    timer3setup_cvargas();
    init();
    
    // make sure everything is off rn
    timerswitch(false);
    go('o');
    m_wait(100); //to show successful startup
    m_red(OFF);
    m_green(OFF);//will come on before this if the wii_setup succeeded.
    
    while (1) {
        if (test) {
            drivetest();
        } else {
            //do localization here
        }
    }
    return 0;   /* never reached */
}

void wii_setup(){
    char sensoron = m_wii_open();
    if(sensoron){
        m_green(ON); //green light on if the sensor connected properly
    }
}

void wii_read(){
    char didreadwork_char = m_wii_read(* data);
    if (didreadwork_char == '1') {
        read_success  = true;
    } else {
        read_success = false;
    }
}

void timer1setup_cvargas(int time_scale){
    
    //turn clock on and scale by the time_scale input.
    //defaults to scale by 1
    switch (time_scale) {
        case 0:
            clear(TCCR1B, CS10);
            clear(TCCR1B, CS11);
            clear(TCCR1B, CS12);
            break;
        case 1:
            set(TCCR1B, CS10);
            clear(TCCR1B, CS11);
            clear(TCCR1B, CS12);
            break;
        case 8:
            clear(TCCR1B, CS10);
            set(TCCR1B, CS11);
            clear(TCCR1B, CS12);
            break;
        case 64:
            set(TCCR1B, CS10);
            set(TCCR1B, CS11);
            clear(TCCR1B, CS12);
            break;
        case 256:
            clear(TCCR1B, CS10);
            clear(TCCR1B, CS11);
            set(TCCR1B, CS12);
            break;
        case 1024:
            set(TCCR1B, CS10);
            clear(TCCR1B, CS11);
            set(TCCR1B, CS12);
            break;
        default:
            set(TCCR1B, CS10);
            clear(TCCR1B, CS11);
            clear(TCCR1B, CS12);
            break;
    }
    
    //set to:
    // (mode 15) UP to OCR1A, PWM mode
    set(TCCR1A, WGM10);
    set(TCCR1A, WGM11);
    set(TCCR1B, WGM12);
    set(TCCR1B, WGM13);
    
    //set B6 for output
    set(DDRB, 6);
    
    //set OCR1B to the switch_voltage input after checking to make sure input is valid
//    if ((trigger > 0x00FF) || (trigger < 0x0000)) {
//        trigger = 127;
//    }
//    OCR1B = trigger;
    
    //configure timer to:
    //clear at OCR1B
    //this sets and clears output of pin B6
    //sets at rollover
    set(TCCR1A,COM1B1);
    clear(TCCR1A,COM1B0);
    
    //set default counter values
    OCR1B = 20000;
    OCR1A = 40000; //PWMing at 400 Hz rn
}

void timer3setup_cvargas(){
    
    //turn clock on and scale by the time_scale input.
    //defaults to scale by 0
    clear(TCCR3B, CS32);
    clear(TCCR3B, CS31);
    clear(TCCR3B, CS30);
    
    //set to: (mode 14) UP to ICR3, PWM mode
    // UP to ICR3, PWM mode
    set(TCCR3B,WGM33);
    set(TCCR3B, WGM32);
    set(TCCR3A, WGM31);
    clear(TCCR3A, WGM30);
    
    //set C6 for output
    set(DDRC, 6);
    
    //configure timer to:
    //clear at OCR3A
    //this sets and clears output of pin C6
    //sets at rollover
    set(TCCR3A, COM3A1);
    clear(TCCR3A, COM3A0);
    
    //set default counter values
    OCR3A = 20000; //for PWM of motor 2... C6
    ICR3 = 40000; //PWMing at 400 Hz rn
}

void timerswitch(bool on){
    if(on){
        set(TCCR3B, CS30); //start timer3
        set(TCCR1B, CS10); //start timer1
    } else {
        clear(TCCR3B, CS30); //stop timer3
        clear(TCCR1B, CS10); //stop timer1
        clear(motor2, PWM2); //make sure the PWM pin is low
        clear(motor1, PWM1); //mkae sure the PWM pin is low
    }
}

void init(){
    m_clockdivide(0);

    //set direction pins for output
    //set(DDRx,n); //PWM PIN for timer1 covered by B6 in timer1setup_cvargas
    // PWM PIN for timer4 also covered in C7 in timer4setup_cvargas
    set(DDRB, direction1);
    set(DDRC, direction2);
    set(DDRB, enableNUM);
    
    //bring actual outputs low
    clear(enablePIN, enableNUM);
    
    //enable global interrupts
    sei();
}

void go(char direction){
//    int i = 0; // for m_wait time function later
    switch (direction) {
        case 'l':
            clear(motor1, direction1);//motor1 forward
            set(motor2, direction2);//motor2 backwards
            timerswitch(true);//start the pwm
            set(enablePIN, enableNUM);//set the enable line high
            break;
        case 'r':
            set(motor1, direction1);//motor1 backwards
            clear(motor2, direction2);//motor2 forward
            timerswitch(true); //start the pwm
            set(enablePIN, enableNUM);//set the enable line high
            break;
        case 'f':
            //both motors same direction
            set(motor1, direction1);
            set(motor2, direction2);
            timerswitch(true); //start the pwm
            set(enablePIN, enableNUM);//set the enable line high
            break;
        case 'b':
            //both motors same direction but opposite of 'f'
            clear(motor1, direction1);
            clear(motor2, direction2);
            timerswitch(true); //start the pwm
            set(enablePIN, enableNUM);//set the enable line high
            break;
        case 'o':
            timerswitch(false);//disable both PWMs
            clear(enablePIN, enableNUM);//clear the enable line low
            clear(motor1, PWM1);//make double sure that the PWM output is cleared for both lines
            clear(motor2, PWM2);
            break;
        default:
            m_green(ON);
            break;
    }
}

void drivetest(){
    go('f');
    m_wait(1000);
    go('b');
    m_wait(1000);
    go('l');
    m_wait(1000);
    go('r');
    m_wait(1000);
    go('o');
    m_wait(1000);
}