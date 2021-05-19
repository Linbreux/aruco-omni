#ifndef SOFTPWM_H
#define SOFTPWM_H

// motor 1
extern const int PWM1;
extern const int INI1_1;
extern const int INI2_1;

// motor 2
extern const int PWM2;
extern const int INI1_2;
extern const int INI2_2;

// motor 3
extern const int PWM3;
extern const int INI1_3;
extern const int INI2_3;

typedef struct Coordinaat coordinaat;

extern double M1;
extern double M2;
extern double M3;

extern double hoek;
// hulpvar voor goneometrie
extern double z;


int Init_Motor_Control(void);

void Motor_Control(int motor_nr,int velocity, int direction);

void Set_Motor(int motor, int signed_pwm);

void set_motor_speed(int motor_nr, double speed);

void set_robot_speed(double x_speed, double y_speed, double z_speed);

int setZwaartepunt(double x, double y, double hoekZW);

#endif