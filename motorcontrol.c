#include "motorcontrol.h"
#include <pigpio.h>
#include <math.h>
#include <stdio.h>


// motor 1
const int PWM1 = 19;
const int INI1_1 = 23;
const int INI2_1 = 24;

// motor 2
const int PWM2 = 13;
const int INI1_2 = 6;
const int INI2_2 = 5;

// motor 3
const int PWM3 = 20;
const int INI1_3 = 11;
const int INI2_3 = 9;




int Init_Motor_Control(void){
    
	if (gpioInitialise() < 0) return 1;
	// Initialisation of outputs
	gpioSetPWMrange(PWM1, 255);
	gpioSetMode(PWM1, PI_OUTPUT);
	gpioSetMode(INI1_1, PI_OUTPUT);
	gpioSetMode(INI2_1, PI_OUTPUT);
	
	gpioSetPWMrange(PWM2, 255);
	gpioSetMode(PWM2, PI_OUTPUT);
	gpioSetMode(INI1_2, PI_OUTPUT);
	gpioSetMode(INI2_2, PI_OUTPUT);
    
	gpioSetPWMrange(PWM3, 255);
	gpioSetMode(PWM3, PI_OUTPUT);
	gpioSetMode(INI1_3, PI_OUTPUT);
	gpioSetMode(INI2_3, PI_OUTPUT);

	 
	return 0;
}


 void Motor_Control(int motor_nr,int velocity, int direction){
	velocity *= 3; //snelheid maal 3 doen
	int PinPwm , PinOut1 , PinOut2 ;
	
	if (motor_nr == 1) {
		 PinPwm = PWM1 ;
		 PinOut1 = INI1_1;
		 PinOut2 = INI2_1;
	} else if(motor_nr == 2) {
		 PinPwm = PWM2;
		 PinOut1 = INI1_2;
		 PinOut2 = INI2_2;
	} else if(motor_nr == 3){
		 PinPwm = PWM3;
		 PinOut1 = INI1_3;
		 PinOut2 = INI2_3;
	}

	if (direction == 1){
		gpioWrite(PinOut1,1);
		gpioWrite(PinOut2,0);
	} else if (direction == -1){
		gpioWrite(PinOut1,0);
		gpioWrite(PinOut2,1);
	} else {
		gpioWrite(PinOut1,0);
		gpioWrite(PinOut2,0);
	}


	//func(GPIO_pin, freq, duty)
	//PWMfreq: 0 (off) or 1-125M (1-187.5M for the BCM2711)
	//PWMduty: 0 (off) to 1000000 (1M)(fully on
	if (velocity>= 255){
		velocity = 255;
	}
	else if (velocity <= -255){
		velocity = -255;
	}
	gpioPWM(PinPwm,velocity);
}
void Set_Motor(int motor, int signed_pwm){
	
	//printf("Waarde van signed_pwm= %d\n",signed_pwm);
	
	int mypwm = signed_pwm;
	double sturing;
	int direction = 1;
	if (mypwm < 0 ) {
		mypwm = -mypwm;
		direction = -1;
	} else if (mypwm ==0) {
		direction = 0 ;
	}
	sturing = (242.0/255.0)*mypwm+13;
	Motor_Control(motor, (int)sturing, direction);
}

void set_motor_speed(int motor_nr, double speed){
	Set_Motor(motor_nr, (int)speed);
}

//structure voor coördinaten
typedef struct Coordinaat{
	double x;
	double y;
}coordinaat;

coordinaat mid;

double M1;
double M2;
double M3;

double XSPEED, YSPEED, ZSPEED;


void set_robot_speed(double x_speed, double y_speed, double z_speed){
	//printf("x = %lf, y = %lf \n", x_speed, y_speed);
	double hoekB, hoekY, hoekX;
	// Afstand van wielen tot middenpunt
	const double afstand_wielen_tot_mid = 4.7;

	// Afstanden van motor tot zwaartepunt
	double motor1_afstand; 
	double motor2_afstand;
	double motor3_afstand; 

	coordinaat motor1_mid, motor2_mid, motor3_mid;
	motor1_mid.x = afstand_wielen_tot_mid * cos(30.0*(M_PI/180));
	motor1_mid.y = afstand_wielen_tot_mid * sin(30.0*(M_PI/180));
	
	motor2_mid.x = afstand_wielen_tot_mid * cos(150.0*(M_PI/180));
	motor2_mid.y = afstand_wielen_tot_mid * sin(150.0*(M_PI/180));

	motor3_mid.x = afstand_wielen_tot_mid * cos(270.0*(M_PI/180));
	motor3_mid.y = afstand_wielen_tot_mid * sin(270.0*(M_PI/180));

	//stel zwaartepunt in voor z rotatie
	coordinaat motor1_co, motor2_co, motor3_co;

	motor1_co.x = -mid.x + motor1_mid.x;
	motor1_co.y = -mid.y + motor1_mid.y;
	motor1_afstand = sqrt(pow(motor1_co.x,2) + pow(motor1_co.y,2));

	motor2_co.x = -mid.x + motor2_mid.x;
	motor2_co.y = -mid.y + motor2_mid.y;
	motor2_afstand = sqrt(pow(motor2_co.x,2) + pow(motor2_co.y,2));

	motor3_co.x = -mid.x + motor3_mid.x;
	motor3_co.y = -mid.y + motor3_mid.y;
	motor3_afstand = sqrt(pow(motor3_co.x,2) + pow(motor3_co.y,2));

	// Bereken hoekverschuiving
	hoekB = acos((pow(afstand_wielen_tot_mid,2) + pow(motor1_afstand,2) - pow(z,2))/(2*motor1_afstand*afstand_wielen_tot_mid));
	hoekY = acos((pow(motor2_afstand,2) + pow(afstand_wielen_tot_mid,2) - pow(z,2))/(2*afstand_wielen_tot_mid*motor2_afstand));
	hoekX = atan(mid.x/(afstand_wielen_tot_mid+mid.y));
	//printf("A: %lf B: %lf C: %lf \n",hoekB,hoekY,hoekX);

	// rotatiesnelheid berekenen per motor
	double A = sin(M_PI/2-hoekB)*(motor1_afstand/afstand_wielen_tot_mid)*z_speed;
	double B = sin(M_PI/2-hoekY)*(motor2_afstand/afstand_wielen_tot_mid)*z_speed;
	double C = sin(M_PI/2-hoekX)*(motor3_afstand/afstand_wielen_tot_mid)*z_speed;

	//printf("A: %lf B: %lf C: %lf \n",A,B,C);

	
	// snelheid motoren berekenen aan de hand van inverse matrix. 
	M1 = (double) -1/3*x_speed 	+ 1.0/sqrt(3) * y_speed 	+ 1.0/3* A;
 	M2 = (double) -1/3*x_speed 	- 1.0/sqrt(3) * y_speed 	+ 1.0/3* B;
	M3 = (double) 2/3*x_speed 	+ 0 * y_speed 				+ 1.0/3* C;

	// print motorsneldheden in m/s
	double v_m1 = M1 * 0.07;
	double v_m2 = M2 * 0.07;
	double v_m3 = M3 * 0.07;
	//printf("m1: %lf m/s  m2: %lf m/s  m3: %lf m/s\n\n",v_m1,v_m2,v_m3);
	printf("m1: %lf    m2: %lf    m3: %lf   \n",M1, M2, M3);

	// stuur de output van elke motor aan.
	set_motor_speed(1, M1);
	set_motor_speed(2, M2);
	set_motor_speed(3, M3);
}

double hoek;
// hulpvar voor goneometrie
double z;

int setZwaartepunt(double x, double y, double hoekZW){
	mid.x = x;
	mid.y = y;
	hoek = hoekZW;
	z = sqrt(pow(mid.x,2)+pow(mid.y,2));
	printf("\n\n x: %f , y: %f ,z: %f\n",x,y,z);
}
