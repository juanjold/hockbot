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
#include "m_bus.h"
#include "m_rf.h"


//just changed motors, 1 and 2 are now c and b respectively
#define motor1 PORTC
#define direction1 7 //high is forward and low is backwards
#define PWM1 6 //this is the PIN on register that will be used to PWM the motor enable... output of timer1

#define motor2 PORTB
#define direction2 7 //high is forward and low is backwards
#define PWM2 6

#define enablePIN PORTB
#define enableNUM 0 //B0 for enable on the motor driver. enables both motors

#define timersupto 40000
#define threshold 100  //experiment
#define puck_find_threshold 200

#define epsilon 20 //experiment

#define CHANNEL 1
#define RXADDRESS 0x08 //0x09, 0x0A
#define PACKET_LENGTH 10

char buffer[PACKET_LENGTH] = {0,0,0,0,0,0,0,0,0,0};
//declare variabless
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
double Rot[3][3]; double T[3][3]; double pos[3][1];
double position_current[2];

// directions
typedef enum  {
	LEFT,
	RIGHT,
	UP,
	DOWN
} Direction;

Direction goal = LEFT; //must change acording to switch.

// gaming states
typedef enum{
	LOOKING_FOR_PUCK,
	POINT_TO_PUCK,
	GO_TO_PUCK,
	WITH_PUCK,
	CONTROL,
	PAUSE,
	HALFTIME,
	GAME_OVER,
	PLAY,
	COMM_TEST
} State;

State state = LOOKING_FOR_PUCK; //change this? //experiment

//Sensing
//eyes
volatile int back_right = 0;
volatile int back_left = 0;
volatile int front_right = 0;
volatile int front_left = 0;
volatile int puck_eyes = 0;
volatile int puck_eyes_2 = 0;
volatile int puck_eyes_3 = 0;
volatile int puck_eyes_add = 0;

typedef enum{
	F0,
	F1,
	F4,
	F5,
	B4,
	B5 // left eye
} ADC_PIN;
ADC_PIN adc = F0;

void wii_setup(){
	char sensoron = m_wii_open();
	m_usb_tx_string("is sensor on? ");
	m_usb_tx_int(sensoron);
	m_usb_tx_string("\n");
	if(sensoron == 1) {
		set(PORTF,6);
		m_wait(2000);
		clear(PORTF,6);
	} else {
			m_green(ON);
			set(PORTF,6);
			m_wait(2000);
			clear(PORTF,6);
			m_wait(1000);
			set(PORTF,6);
			m_wait(2000);
			clear(PORTF,6);
			m_wait(1000);
			set(PORTF,7);
			m_wait(2000);
			clear(PORTF,7);
			m_wait(1000);
			set(PORTF,6);
			m_wait(2000);
			clear(PORTF,6);
			m_wait(1000);
			set(PORTF,6);
			m_wait(2000);
			clear(PORTF,6);
			m_wait(1000);
			m_green(OFF);
	}
	//m_usb_tx_uint(sensoron);
	//m_usb_tx_string(" sensor on \n" );
}

void wii_read(){
	char didreadwork_char = m_wii_read( data ); //NOT SURE IF THIS IS RIGHT
	//m_usb_tx_uint(didreadwork_char);
	int i;
	/*for (i = 0; i < 12; i++){
	m_usb_tx_uint(data[i]);
	m_usb_tx_string(" ");
	}
	m_usb_tx_string("read \n" )*/;
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
	/*
	m_usb_tx_string(" BEFORE " );
	m_usb_tx_string(" vect_bottomtotop[0]: " );
	m_usb_tx_int(vect_bottomtotop[0]);
	m_usb_tx_string(" vect_bottomtotop[1]: " );
	m_usb_tx_int(vect_bottomtotop[1]);
	*/
	vect_bottomtotop[0] = vect_bottomtotop[0] / magnitude;
	vect_bottomtotop[1] = vect_bottomtotop[1] / magnitude;
	
	//find the dot product of the two vectors
	dot = up[0]*vect_bottomtotop[0] + up[1]*vect_bottomtotop[1]; //LOLz is this how you take a dot product or nah
	
	theta = heresyoursign * acos(dot);
	
	/*
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
	*/
}

void calculate_rotation() {
	//    double R[2][2] = { {cos(theta), -1*sin(theta)}, {sin(theta), cos(theta)} };
	/*	Rot[0][0] = cos(theta); Rot[0][1] = -1*sin(theta); Rot[0][2] = 0;
	Rot[1][0] = sin(theta); Rot[1][1] = cos(theta);	   Rot[1][2] = 0;
	Rot[2][0] = 0;		  ; Rot[2][1] = 0;			   Rot[2][2] = 1;
	
	T[0][0] = 0;	T[0][1] = 0;	T[0][2] = pvect[0];
	T[1][0] = 0;	T[1][1] = 0;	T[1][2] = pvect[1];
	T[2][0] = 0;	T[2][1] = 0;	T[2][2] = 1;
	
	//pos = T * Rot * C
	position_current[0] = cent[0]*(Rot[0][0]*T[0][0] + Rot[0][1]*T[1][0] + Rot[0][2]*T[2][0]) + cent[1]*(Rot[0][0]*T[0][1] + Rot[0][1]*T[1][1] + Rot[0][2]*T[2][1]) + (Rot[0][0]*T[0][2] + Rot[0][1]*T[1][2] + Rot[0][2]*T[2][2]);
	position_current[1] = cent[0]*(Rot[1][0]*T[0][0] + Rot[1][1]*T[1][0] + Rot[1][2]*T[2][0]) + cent[1]*(Rot[1][0]*T[0][1] + Rot[1][1]*T[1][1] + Rot[1][2]*T[2][1]) + (Rot[1][0]*T[0][2] + Rot[1][1]*T[1][2] + Rot[1][2]*T[2][2]);
	//pos[2] = 1;
	*/
	
	R[0][0] = -cos(theta);
	R[0][1] = sin(theta);
	R[1][0] = -sin(theta);
	R[1][1] = -cos(theta);
	
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

void init_ADC()
{
	//voltage ref Vcc
	clear(ADMUX,REFS1);
	set(ADMUX,REFS0);
	
	//prescaler to /128
	set(ADCSRA,ADPS2);
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);
	
	//disabling input
	set(DIDR0,ADC0D);
	set(DIDR0,ADC1D);
	set(DIDR0,ADC4D);
	set(DIDR0,ADC5D);
	
	set(DIDR2,ADC11D);
	
	//channel sel
	clear(ADCSRB,MUX5);
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	
	//enable interrupts
	set(ADCSRA,ADIE);
}

void init(){
	m_clockdivide(0);
	m_disableJTAG();
	//set direction pins for output
	//set(DDRx,n); //PWM PIN for timer1 covered by B6 in timer1setup_cvargas
	// PWM PIN for timer4 also covered in C7 in timer4setup_cvargas
	set(DDRB, direction1);
	set(DDRC, direction2);
	set(DDRB, enableNUM);
	
	//solenoid
	set(DDRD,7);
	clear(PORTD,7);
	
	//LED 1 - bLUE
	set(DDRF,6);
	clear(PORTF,6);
	
	//LED 2 - RED
	set(DDRF,7);
	clear(PORTF,7);
	
	//DIP switches //trying b1
	clear(DDRD,5); //switch1
	clear(DDRD,6); //switch2
	//clear(PORTD,5);
	//clear(PORTD,6);
	
	//ADC
	clear(DDRF,0);
	clear(DDRF,1);
	clear(DDRF,4);
	clear(DDRF,5);
	clear(DDRB,4);
	//bring actual outputs low
	clear(enablePIN, enableNUM);
	init_ADC();
	//enable global interrupts
	sei();

	timer1setup_cvargas(0);
	timer3setup_cvargas();
	m_usb_init();
	// configure mRF
	m_bus_init();
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
}

int main(void){
	//call the external setup functions
	m_red(ON);
	init();
	m_red(OFF);
	m_rf_open(CHANNEL,RXADDRESS,PACKET_LENGTH);
	// make sure everything is off rn
	timerswitch(false);
	go('o');
	m_wait(100); //to show successful startup
	//while(!m_usb_isconnected()); // REMEMBER TO GET RID OF THIS YO
	m_usb_tx_string("finished setup \n");
	m_green(OFF);//will come on before this if the wii_setup succeeded.
	m_wait(250);
	m_usb_tx_string("entering wii setup\n");
	wii_setup();
	m_wait(250);
	m_usb_tx_string("in while loop\n" );
	
	//start ADC conversion
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC);
	m_wait(100);
	//set goal
	int led = 7;
	//   if(check(PIND,5)) //change to actual pin // PIN D5 sets the goal
	//    {
	//blue LED indicates right goal
	goal = RIGHT;
	set(PORTF,7);//blue
	m_wait(2000);
	clear(PORTF,7);

	while (1) {
		switch(state) {
			case LOOKING_FOR_PUCK:
			//do something. for now, rotate in place.
			left_PWM(26);
			right_PWM(26);
			go('l');
					/*	m_usb_tx_string("F0: ");
						m_usb_tx_int(back_left);
						m_usb_tx_string("    | F1: ");
						m_usb_tx_int(back_right);
						m_usb_tx_string("    | F4: ");
						m_usb_tx_int(front_left);
						m_usb_tx_string("    | F5: ");
						m_usb_tx_int(front_right);
						m_usb_tx_string("    | B5: ");
						m_usb_tx_int(puck_eyes);
						m_usb_tx_string("\n");*/
			if (back_left > threshold || back_right > threshold ||
			front_left > threshold || front_right > threshold)
			{
				//m_usb_tx_string("we found the puck");
				state = POINT_TO_PUCK;
				m_green(ON);
			}
			break;
			
			case POINT_TO_PUCK:
			/*m_usb_tx_string("BL: ");
			m_usb_tx_int(back_left);
			m_usb_tx_string("    | BR: ");
			m_usb_tx_int(back_right);
			m_usb_tx_string("    | FL: ");
			m_usb_tx_int(front_left);
			m_usb_tx_string("    | Fr: ");
			m_usb_tx_int(front_right);
			m_usb_tx_string("    | PEL: ");
			m_usb_tx_int(puck_eyes);
			m_usb_tx_string("\n");
			*/
			//back_right eye, turn right
			if(back_right > back_left && back_right > front_right && back_right > front_left)
			{
				left_PWM(26);
				right_PWM(26);
				go('r');
			}
			//likewise, back_left eye, turn left
			else if (back_left > back_right && back_right > front_right && back_right > front_left)
			{
				left_PWM(26);
				right_PWM(26);
				go('l');
			}
			
			//idk about this threshold //experiment
			else if(front_right > threshold || front_left > threshold)
			{
				//turn right if it's to the right
				if(front_right > front_left)
				{
					left_PWM(26);
					right_PWM(26);
					go('r');
				}
				//
				else if (front_left > front_right)
				{
					left_PWM(26);
					right_PWM(26);
					go('l');
				}
			}
			
			//if absolute value of different between front eyes is smaller than E, and the values
			//are greater than some threshold (take into account when neither eye sees something)
			if((abs(front_right-front_left) < 60) && (front_left > 200 &&  front_right > 200)) //experiment threshold
			{
				m_green(ON);
				m_red(ON);
				state = GO_TO_PUCK;
			}
			break;
			
			case GO_TO_PUCK:
			m_red(ON);
			//if we're still seeing the PUCK
			if((abs(front_right-front_left) < 60) && (front_left > 200 &&  front_right > 200)) {
				left_PWM(26);
				right_PWM(26);
				go('f');
			}
			else
			{
				state = POINT_TO_PUCK;
				go('o');
			}
			//check to see if we have the puck in the sloot
			puck_eyes_add = 0;
			if(puck_eyes   > 900){ puck_eyes_add++;}
			if(puck_eyes_2 > 900){ puck_eyes_add++;}
			if(puck_eyes_3 > 900){ puck_eyes_add++;}
			if(puck_eyes_add >= 2 ) // put in an actual pin to detect puck in sloot
			{
				state = WITH_PUCK;
				puck_eyes_add = 0;
			}
			puck_eyes_add = 0;
			break;
			
			case WITH_PUCK:
			m_green(ON);
			m_red(OFF);
			//turn to goal
			buylocal();
			
			//          UP 0
		/*	 /-----------------------\
			|						 |
		Left|						 | Right
		-90	|						 |	90
			|						 |
			\-----------------------/
			*/
			switch (goal)
			{
				case RIGHT:
				//turn towards theta, in a slow arc.
				//experiment, need to check all these values
				if (abs(theta) < 88)
				{
					left_PWM(28);
					right_PWM(25);
					go('f');
				}
				else if (abs(theta) > 92)
				{
					left_PWM(25);
					right_PWM(28);
					go('f');
				}
				else
				{	//I mean why not? lololol
					left_PWM(50);
					right_PWM(50);
					go('f');
				}
				break;
				
				case LEFT:
				if (-abs(theta) > -88)
				{
					left_PWM(25);
					right_PWM(28);
					go('f');
				}
				else if (-abs(theta) < -92)
				{
					left_PWM(28);
					right_PWM(25);
					go('f');
				}
				else
				{	//I mean why not? lololol
					left_PWM(50);
					right_PWM(50);
					go('f');
				}
				break;
			}
			puck_eyes_add = 0;
			if(puck_eyes   < 900){ puck_eyes_add++;}
			if(puck_eyes_2 < 900){ puck_eyes_add++;}
			if(puck_eyes_3 < 900){ puck_eyes_add++;}
			if(puck_eyes_add >= 2) //put in an actual pin here for puck //experiment
			{
				state = POINT_TO_PUCK;
				puck_eyes_add = 0;
			}
			puck_eyes_add = 0;
			break;
			
			case CONTROL:
			if(buffer[0] == 0xA0) {
				state = COMM_TEST;
				} else if (buffer[0] == 0xA1) {
				state = PLAY;
				} else if (buffer[0] == 0xA2 || buffer[0] == 0xA3){
				state  = PAUSE;
				} else if (buffer[0] == 0xA4) {
				state = PAUSE;
				} else if (buffer[0] == 0xA6) {
				state = HALFTIME;
				} else if (buffer[0] == 0xA7) {
				state = PAUSE;
			}
			break;
			
			case COMM_TEST:
			//flash blue LED
			m_red(ON);
			m_wait(1000);
			clear(PORTF,led);
			m_red(OFF);
			m_wait(1000);
			set(PORTF,led);
			m_red(ON);
			m_wait(1000);
			clear(PORTF,led);
			m_red(OFF);
			state = PAUSE;
			break;
			
			case PLAY:
			//for now just look for puck!
			state = LOOKING_FOR_PUCK;
			/*if(check(PIND,5))
			{
			set(PORTF,6);//red
			}
			else
			{
			set(PORTF,7);//blue
			}*/
			set(PORTF,led);
			break;
			
			case PAUSE:
			/*if(check(PIND,6))
			{
			clear(PORTF,6);//red
			}
			else
			{
			clear(PORTF,7);//blue
			}*/
			clear(PORTF,led);
			go('o');
			left_PWM(0);
			right_PWM(0);
			break;
			
			case HALFTIME:
			go('o');
			left_PWM(0);
			right_PWM(0);
			led = 6;
			break;
		}
	}
	return 0;   /* never reached */
}

ISR(ADC_vect)
{
	switch(adc)	{
		case F0:
		back_left = ADC;
		set  (ADMUX,MUX0);
		clear(ADMUX,MUX1);
		clear(ADMUX,MUX2);
		clear(ADMUX,MUX5);
		adc = F1;
		break;
		
		case F1:
		front_right = ADC;
		clear(ADMUX,MUX0);
		clear(ADMUX,MUX1);
		set  (ADMUX,MUX2);
		clear(ADMUX,MUX5);
		adc = F4;
		break;
		
		case F4:
		front_left = ADC;
		set  (ADMUX,MUX0);
		clear(ADMUX,MUX1);
		set  (ADMUX,MUX2);
		clear(ADMUX,MUX5);
		adc = F5;
		break;
		
		case F5:
		back_right = ADC;
		clear(ADMUX,MUX0);
		clear(ADMUX,MUX1);
		set  (ADMUX,MUX2);
		set  (ADCSRB,MUX5);
		adc = B5;
		break;
		
		case B5:
		
		puck_eyes_3 = puck_eyes_2;
		puck_eyes_2 = puck_eyes;
		puck_eyes = ADC;
		clear(ADMUX,MUX0);
		clear(ADMUX,MUX1);
		clear(ADMUX,MUX2);
		clear(ADCSRB,MUX5);
		adc = F0;
		break;
	}
	//might need to do with flags inside while loop if !working //experiment
	set(ADCSRA,ADEN);
	set(ADCSRA,ADSC);
}

ISR(INT2_vect)
{
	m_rf_read(buffer,PACKET_LENGTH);
	state = CONTROL;
	m_red(ON);
	//save old state??
}