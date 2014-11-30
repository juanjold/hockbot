/* Name: robockey driver 1.0
 * Authors: Cruz, Teddy, Juanjo, & Spencer
 */

/*
 errors that could turn on the green LED:
 
 go() throws if invalid input given
 find_max() throws if no max found
 find_toporbottom() throws if it can't figure out which is top or bottom
 wii_setup() if it failz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "m_general.h"
#include "m_wii.h"
#include "m_usb.h"

#define motor1 PORTB
#define direction1 7 //high is forward and low is backwards
#define PWM1 6 //this is the PIN on register that will be used to PWM the motor enable... output of timer1

#define motor2 PORTC
#define direction2 7 //high is forward and low is backwards
#define PWM2 6

#define enablePIN PORTB
#define enableNUM 0 //B0 for enable on the motor driver. enables both motors

#define timersupto 40000

//declare variables
bool test = false;
unsigned int data[12]; // = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; for storing incoming wii data
bool read_success; //determining whether or not the call to the wii was successfull or nah

//variables for distance determining
//int x1in; int y1in; int x2in; int y2in; int x3in; int y3in; int x4in; int y4in;
int xin[4]; int yin[4];
volatile int d12; volatile int d13; volatile int d14; volatile int d23; volatile int d24; volatile int d34;
volatile int maxd; volatile int maxdp1; volatile int maxdp2; volatile int otherdp1; volatile int otherdp2; // bool maxfound;

//variables for location points
int cent[2]; int point1[2]; int point2[2];

//variables for toporbototm
//bool toporbottomfound = false;
int dm[4][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
int top; int bottom;
volatile int heresyoursign; volatile double vect_bottomtotop[2];

//variables for finding the theta
volatile double theta;
double dot; double magnitude;
double up[2] = {0 , 1}; //a vector pointing straight up

//variables for rotation
double R[2][2]; double pvect[2];
double position_current[2];

void wii_setup(){
    char sensoron = m_wii_open();
    m_usb_tx_uint(sensoron);
    m_usb_tx_string(" sensor on \n" );
}

void wii_read(){
    char didreadwork_char = m_wii_read( data ); //NOT SURE IF THIS IS RIGHT
    //m_usb_tx_uint(didreadwork_char);
    int i;
    for (i = 0; i < 12; i++){
        m_usb_tx_uint(data[i]);
        m_usb_tx_string(" ");
    }
    m_usb_tx_string("read \n" );
    if (didreadwork_char == 1) {
        read_success  = true;
        m_red(ON);
    } else {
        read_success = false;
        m_green(ON);
    }
    //all code below this line assumes that data has 12 elements that are the readings from the wii sensor
    xin[0] = data[0]; xin[1] = data[3]; xin[2] = data[6]; xin[3] = data[9];
    yin[0] = data[1]; yin[1] = data[4]; yin[2] = data[7]; yin[3] = data[10];
}

void find_distances(){
    //find the distances between the points you took in
    d12 = sqrt( pow((xin[1] - xin[0]) , 2) + pow((yin[1] - yin[0]) , 2) );
    d13 = sqrt( pow((xin[2] - xin[0]) , 2) + pow((yin[2] - yin[0]) , 2) );
    d14 = sqrt( pow((xin[3] - xin[0]) , 2) + pow((yin[3] - yin[0]) , 2) );
    d23 = sqrt( pow((xin[2] - xin[1]) , 2) + pow((yin[2] - yin[0]) , 2) );
    d24 = sqrt( pow((xin[3] - xin[1]) , 2) + pow((yin[3] - yin[1]) , 2) );
    d34 = sqrt( pow((xin[3] - xin[2]) , 2) + pow((yin[3] - yin[2]) , 2) );
    
    //fill in the distance matrix
    dm[1][0] = d12; dm[0][1] = d12;
    dm[2][0] = d13; dm[0][2] = d13;
    dm[3][0] = d14; dm[0][3] = d14;
    dm[2][1] = d23; dm[1][2] = d23;
    dm[3][1] = d24; dm[1][3] = d24;
    dm[3][2] = d34; dm[2][3] = d34;
}

void filter_outliers(){
    //TO IMPLEMENT: don't do the calculations if there are less than two points. Just keep swimming.
    
    //if the value in dm came from a 1023 then set hte dm value to NAN
    if(xin[0] == 1023 || yin[0] == 1023){
        dm[0][1] = NAN;
        dm[0][2] = NAN;
        dm[0][3] = NAN;
        dm[1][0] = NAN;
        dm[2][0] = NAN;
        dm[3][0] = NAN;
    }
    if(xin[1] == 1023 || yin[1] == 1023){
        dm[1][0] = NAN;
        dm[1][2] = NAN;
        dm[1][3] = NAN;
        dm[0][1] = NAN;
        dm[2][1] = NAN;
        dm[3][1] = NAN;
    }
    if (xin[2] == 1023 || yin[2] == 1023) {
        dm[2][0] = NAN;
        dm[2][1] = NAN;
        dm[2][3] = NAN;
        dm[0][2] = NAN;
        dm[1][2] = NAN;
        dm[3][2] = NAN;
    }
    if (xin[3] == 1023 || yin[3] == 1023) {
        dm[3][0] = NAN;
        dm[3][1] = NAN;
        dm[3][2] = NAN;
        dm[0][3] = NAN;
        dm[1][3] = NAN;
        dm[2][3] = NAN;
    }
}

void find_max(){
    
    maxd = 0;
    
    if (d12 > maxd) {
        maxd = d12;
        maxdp1 = 1 - 1;
        maxdp2 = 2 - 1;
        otherdp1 = 3 - 1;
        otherdp2 = 4 - 1;
    }
    if (d13 > maxd) {
        maxd = d13;
        maxdp1 = 1 - 1;
        maxdp2 = 3 - 1;
        otherdp1 = 2 - 1;
        otherdp2 = 4 - 1;
    }
    if (d14 > maxd) {
        maxd = d14;
        maxdp1 = 1 - 1;
        maxdp2 = 4 - 1;
        otherdp1 = 2 - 1;
        otherdp2 = 3 - 1;
    }
    if (d23 > maxd) {
        maxd = d23;
        maxdp1 = 2 - 1;
        maxdp2 = 3 - 1;
        otherdp1 = 1 - 1;
        otherdp2 = 4 - 1;
    }
    if (d24 > maxd) {
        maxd = d24;
        maxdp1 = 2 - 1;
        maxdp2 = 4 - 1;
        otherdp1 = 1 - 1;
        otherdp2 = 3 - 1;
    }
    if (d34 > maxd) {
        maxd = d34;
        maxdp1 = 3 - 1;
        maxdp2 = 4 - 1;
        otherdp1 = 1 - 1;
        otherdp2 = 2 - 1;
    }
    if (maxd == 0){m_green(ON);}
}

void find_center(){
    
    point1[0] = xin[maxdp1];
    point1[1] = yin[maxdp1];
    
    point2[0] = xin[maxdp2];
    point2[1] = yin[maxdp2];
    
    cent[0] = (point1[0] + point2[0]) / 2; //Should this be a float or double or something? Yeah?
    cent[1] = (point1[1] + point2[1]) / 2; //Should this be a float or double or something? Yeah?
}

void find_toporbottom(){
    //at this point you know which points are the further apart (maxdp1 & maxdp2)
    //and you have the indexes of the other two points otherdp1 & otherdp2 in no particular order
    
    //find whether maxdp1 or maxdp2 is closer to otherdp1 and other dp2
    //if maxdp1 is closer other point 2 OR if maxdp1 is closer to other point 1 then max dp1 is the top
    if ((dm[maxdp1][otherdp2] < dm[maxdp2][otherdp2]) || (dm[maxdp1][otherdp1] < dm[maxdp2][otherdp2]) ){
        //then maxdp1 is the top
        top = maxdp1;
        bottom = maxdp2;
    } else if( (dm[maxdp2][otherdp2] < dm[maxdp1][otherdp2]) || (dm[maxdp2][otherdp1] < dm[maxdp1][otherdp2]) ){
        top = maxdp2;
        bottom = maxdp1;
    } else {
        m_green(ON);
    }
    
    //get the sign
    if(xin[top] < xin[bottom]){
        heresyoursign = -1;
    } else {
        heresyoursign = 1;
    }
    
    //lol like probably right. supposed to be a vector from bottom to top as the name implies. CHECK index numbers
    vect_bottomtotop[0] = xin[top] - xin[bottom];
    vect_bottomtotop[1] = yin[top] - yin[bottom];
}

void calculate_thetas(){
//    thetas(i) = heresyoursign*acos( dot(up, (vect/norm(vect))) ); %MATLAB code
    
    //make the vect_bottomtotop a unit vector
    magnitude = sqrt( pow(vect_bottomtotop[0],2) + pow(vect_bottomtotop[1],2) );
    
    m_usb_tx_string(" BEFORE " );
    m_usb_tx_string(" vect_bottomtotop[0]: " );
    m_usb_tx_int(vect_bottomtotop[0]);
    m_usb_tx_string(" vect_bottomtotop[1]: " );
    m_usb_tx_int(vect_bottomtotop[1]);
    
    vect_bottomtotop[0] = vect_bottomtotop[0] / magnitude;
    vect_bottomtotop[1] = vect_bottomtotop[1] / magnitude;
    
    //find the dot product of the two vectors
    dot = up[0]*vect_bottomtotop[0] + up[1]*vect_bottomtotop[1]; //LOLz is this how you take a dot product or nah
    
    theta = heresyoursign * acos(dot);
    
    m_usb_tx_string(" AFTER " );
    m_usb_tx_string(" heresyoursign: " );
    m_usb_tx_int(heresyoursign);
    m_usb_tx_string(" madnitude: " );
    m_usb_tx_int(magnitude);
    m_usb_tx_string(" dot: " );
    m_usb_tx_int(dot);
    m_usb_tx_string(" vect_bottomtotop[0]: " );
    m_usb_tx_int(vect_bottomtotop[0]);
    m_usb_tx_string(" vect_bottomtotop[1]: " );
    m_usb_tx_int(vect_bottomtotop[1]);
    m_usb_tx_string("  ");
}

void calculate_rotation() {
//    double R[2][2] = { {cos(theta), -1*sin(theta)}, {sin(theta), cos(theta)} };
    R[0][0] = cos(theta);
    R[0][1] = -1*sin(theta);
    R[1][0] = sin(theta);
    R[1][1] = cos(theta);
    
    //position vector of robot from origin with (0,0) at rink center
    pvect[0] = 512 - cent[0];
    pvect[1] = 384 - cent[1];
    
    //the positions of the robot in terms of the rink... not CM yet actually not really sure what these units would be.
    position_current[0] = R[0][0]*pvect[0] + R[0][1]*pvect[1];//THE X POSITION AS;LDKFJALS;DKFJA;LSKDJF
    position_current[1] = R[1][0]*pvect[0] + R[1][1]*pvect[1];//the Y position
}

void buylocal() {
    wii_read();
    find_distances(); //find all combinations of distances
    filter_outliers(); //set dm[][] that came from 1023 point to NAN
    find_max(); //stores points with maximum distance as maxdp1 & maxdp2 STORES INDEXES FOR xin yin (i.e. point# -1)
    find_center(); // sets cent[0] && cent[1] CAMERA CENTER
    find_toporbottom();//find which point is top and which point is bottom (of the two points set by find_max), and makes a vector betwee them, vect_bottomtotop
    calculate_thetas();//find the thetas (from the vectors and stuff) also gives the theta the correct(?) sign
    
    //the error starts here
    calculate_rotation();//matrix multiplication to spin the matrix (lol)
    
    m_usb_tx_int(position_current[0]);
    m_usb_tx_string(",");
    m_usb_tx_int(position_current[1]);
    m_usb_tx_string(" theta: ");
    m_usb_tx_int((theta*180)/M_PI);
    m_usb_tx_string("\n");
    //current positions are now set in position_current[] vector
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
        case 'r':
            set(motor1, direction1);//motor1 forward
            clear(motor2, direction2);//motor2 backwards
            timerswitch(true);//start the pwm
            set(enablePIN, enableNUM);//set the enable line high
            break;
        case 'l':
            clear(motor1, direction1);//motor1 backwards
            set(motor2, direction2);//motor2 forward
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

void left_PWM(int percent){
    if (percent >= 0 && percent <= 100) {
        OCR3A = (timersupto * percent) / 100;
    } else {
        m_green(ON);
        //default to 50% duty cycle
        OCR3A = timersupto / 2; //for PWM of motor 2... C6
        ICR3 = timersupto; //PWMing at 400 Hz rn i.e. this is 40 000
    }
} //give it a percent you want to PWM the motor

void right_PWM(int percent){
    if (percent >= 0 && percent <= 100) {
        OCR1B = (timersupto * percent) / 100;
    } else {
        m_green(ON);
        //default to 50% duty cycle
        OCR1B = timersupto / 2; //for PWM of motor 1
        OCR1A = timersupto; //PWMing at 400 Hz rn i.e. this is 40 000
    }
} //give it a percent you want to PWM the motor

void drivetest(){
    go('f');
    m_wait(1000);
    right_PWM(10);
    m_wait(1000);
    right_PWM(50);
    m_wait(1000);
    right_PWM(75);
    m_wait(1000);
    right_PWM(100);
    m_wait(1000);
    
    go('o');
    m_wait(1000);
    right_PWM(1);
    
    go('f');
    m_wait(1000);
    left_PWM(10);
    m_wait(1000);
    left_PWM(50);
    m_wait(1000);
    left_PWM(75);
    m_wait(1000);
    left_PWM(100);
    m_wait(1000);
//    right_PWM(50)
//
//    go('f');
//    m_wait(500);
//    left_PWM(10);
//    m_wait(500);
//    left_PWM(75);
//    m_wait(500);
//    left_PWM(100);
//    m_wait(500);
}

int main(void)
{
    //call the external setup functions
    m_red(ON);
    init();
    timer1setup_cvargas(0);
    timer3setup_cvargas();
    m_usb_init();
    // make sure everything is off rn
    timerswitch(false);
    go('o');
    m_wait(100); //to show successful startup
    while(!m_usb_isconnected()); // REMEMBER TO GET RID OF THIS YO
    m_usb_tx_string("finished setup \n");
    m_red(OFF);
    m_green(OFF);//will come on before this if the wii_setup succeeded.
    
    m_wait(250);
    m_usb_tx_string("entering wii setup\n");
    wii_setup();
    m_wait(250);
    
    m_usb_tx_string("in while loop\n" );
    while (1) {
        if (test) {
            drivetest();
        } else {
            //do localization here
            buylocal();
            m_wait(250);
            m_red(OFF);
            m_green(OFF);
            m_wait(250);
        }
    }
    return 0;   /* never reached */
}