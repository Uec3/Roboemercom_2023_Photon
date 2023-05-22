#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


void dmpReady();
void Gyro_init(MPU6050 &mpu);
double Get_angle(MPU6050 &mpu);
//------------------------------------------------------------------------------------------------------------------------------------------------------------

class US
{
public:
	US();
	US(int pin);
	US(int echo, int trig);
	int distance();
private:
	long duration;
	int l_duration;
	long f_duration;
	int echo_pin;
	int trig_pin;
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------

class Motor
{
public:
	Motor();
	Motor(int right_pin, int left_pin, int speed_pin, int spd = 0);
	void start_rotation();
	void Set_speed(int spd);
	void Stop();
private:
	int speed;
	int r_pin;
	int l_pin;
	int s_pin;
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------

class PID{
private:
	double Pk,Ik,Dk, o_error = 0, Mi;
	double io_error = 0;
	double p,i,d;
public:
	void rm_e();
	double P();
	double I();
	double D();
	PID();
	PID(double p, double i, double d, double mi);
	double ID(double error);
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------

class Light_sensor
{
public:
	Light_sensor();
	Light_sensor(uint8_t p);
	int Get_data();
	void init();
private:
	uint8_t pin; 	
	int sensor;
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------

class Task
{
public:
	Task(Motor &rm, Motor &lm, MPU6050 &m, PID &mr,PID &se, US &l_us, US &c_us, US &r_us);
	~Task();
	void rta();
	void mta();
	void uia(double a);
	void uta(double a);
	void utd(int a);
	void uzd(int a);
	int gta();
	int gtd();
	void zero();
	int check();
	bool Complete();
	double* pid();
	void mtnc();
	void Stop();
	char Condition();
	// int* usd();
	int z_dst;
private:
	// int* usd_arr;
	int kek;
	char condition;
	int t_dist = 0;
	double sum = 0;
	int counter = 0;
	double* arr = new double[3];
	bool complete = 0;
	double i_angle = 0, t_angle = 0;
	Motor r_motor, l_motor;
	MPU6050 MPU;
	PID motor_rotation, speed_rotation;
	// Light_sensor light_sensor;
	US l_US,r_US, c_US; 
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------
