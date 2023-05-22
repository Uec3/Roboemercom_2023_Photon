#include <Arduino.h>
#include "ground_control.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

//------------------------------------------------------------------------------------------------------------------------------------------------------------

uint8_t fifoBuffer[45];
volatile bool mpuFlag = false;
const int min_spd = 0;
const int max_spd = 150;
void dmpReady(){
	mpuFlag = true;
}
void Gyro_init(MPU6050 &mpu){
	Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(1, dmpReady, RISING);
  mpu.setXAccelOffset(-910);
	mpu.setYAccelOffset(-553);
	mpu.setZAccelOffset(792);
	mpu.setXGyroOffset(178);
	mpu.setYGyroOffset(-23);
	mpu.setZGyroOffset(2);
  // delay(3000);
}
double Get_angle(MPU6050 &mpu){
  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer) && mpuFlag){
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   	  return degrees(ypr[0]);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------

US::US(){
	echo_pin = NULL;	
}
US::US(int pin){
	echo_pin = pin;
	trig_pin = pin;
}

US::US(int echo, int trig){
	echo_pin = echo;
	trig_pin = trig;
}
int US::distance(){
	duration = 0;
	f_duration = 0;
	for(int i = 0; i < 10; i++){
		pinMode(echo_pin, OUTPUT);
		digitalWrite(echo_pin, LOW);
		delayMicroseconds(2);
		digitalWrite(echo_pin, HIGH);
		delayMicroseconds(10);
		digitalWrite(echo_pin, LOW);
		pinMode(echo_pin, INPUT);
		duration = pulseIn(echo_pin, HIGH);
		if(abs((l_duration - duration)) < 40 * 2 / 0.034){
				f_duration += duration;
				// l_duration = duration;
			}
		else{
			f_duration += l_duration;
		}
		l_duration = duration;
	}
	// l_duration = f_duration / 10;
	return f_duration * 0.034/20  ;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------

Motor::Motor(){
	r_pin = 14;
	l_pin = 14;
	s_pin = 14;
	speed = 0;	
}
Motor::Motor(int right_pin, int left_pin, int speed_pin, int spd = 0){
	r_pin = right_pin;
	l_pin = left_pin;
	s_pin = speed_pin;
	speed = spd;
	pinMode(r_pin,OUTPUT);
	pinMode(l_pin,OUTPUT);
	pinMode(s_pin,OUTPUT);
}
void Motor::start_rotation(){
	if(speed > 0){
		digitalWrite(r_pin,1);
		digitalWrite(l_pin,0);
		analogWrite(s_pin,speed);
	}
	else if (speed < 0){
		digitalWrite(r_pin,0);
		digitalWrite(l_pin,1);
		analogWrite(s_pin, -speed);
		
	}
}
void Motor::Set_speed(int spd){
	if(spd >= max_spd){
		speed = max_spd;
	}
	else if (spd <= -max_spd){
		speed = -max_spd;
	}
	else{
		speed = spd;
	}
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
Light_sensor::Light_sensor(uint8_t p){
	pin = p;
	pinMode(pin, INPUT);
}  

void Light_sensor::init(){
	sensor = analogRead(pin);
}

int Light_sensor::Get_data(){
	return analogRead(pin) - sensor;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------

PID::PID(){
	Pk = 0;
	Ik = 0;
	Dk = 0;
	Mi = 0;

}
PID::PID(double p, double i, double d, double mi){
	Pk = p;
	Ik = i;
	Dk = d;
	Mi = mi;
}
void PID::rm_e(){
	io_error = 0;
	o_error = 0;
}
double PID::P(){
	return p;
}
double PID::I(){
	return i;
}
double PID::D(){
	return d;
}
double PID::ID(double error){
	double i_error = error + io_error;
	// double m_error = Mi / Ik;
	if(i_error * Ik > Mi ){
		i_error = Mi / Ik;
	}
	else if(i_error * Ik < -Mi){
		i_error = -Mi / Ik;
	}
	double d_error = error - o_error;
	io_error = i_error;
	o_error = error;
	p = error * Pk;
	i = i_error;
	d = d_error * Dk;
	return error * Pk + i_error * Ik - d_error * Dk;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------

Task::Task(Motor &rm, Motor &lm, MPU6050 &m, PID &mr, PID &se, US &l_us, US &c_us, US &r_us){
	r_motor = rm;
	l_motor = lm;
	MPU = m;
	motor_rotation = mr;
	speed_rotation = se;
	l_US = l_us;
	c_US = c_us;
	r_US = r_us;
}
void Task::uia(double a){
	i_angle = a;
}
void Task::rta(){
	r_motor.Set_speed(motor_rotation.ID(int(i_angle + 540 - t_angle) % 360 - 180));
	l_motor.Set_speed(- motor_rotation.ID(int(i_angle + 540 - t_angle) % 360 - 180));
	r_motor.start_rotation();
	l_motor.start_rotation();
}
bool Task::Complete(){
	return complete;
}
void Task::zero(){
	complete = 0;
}
void Task::uta(double a){
	condition = 'r';
 	sum += i_angle;
 	counter++;
 	zero();
 	if(counter >= 20){
 		counter = 0;
 		sum = 0;
 	}
 	if(abs(int(sum / counter + 540 - t_angle) % 360 - 180) < 2){
 		if(a < -180){
 			t_angle = a + 360;	
 		}
 		else if( a > 180){
 			t_angle = a - 360;
 		}
 		else{
 			t_angle = a;
 		}
 		complete = 1;
 	}
}
int Task::gta(){
 	return t_angle;
}
int Task::gtd(){
	return t_dist;
}
int Task::check(){
	return kek;
}
void Task::Stop(){
	r_motor.Set_speed(0);
	l_motor.Set_speed(0);
	r_motor.start_rotation();
	l_motor.start_rotation();
}
void Task::mtnc(){
	condition = 'm';
	if(c_US.distance() > 25){
		t_dist = 25 - (z_dst - c_US.distance());
	}
	else{
		t_dist = c_US.distance() - 6;
	}
	int spd = speed_rotation.ID(t_dist);
	
	r_motor.Set_speed(spd + motor_rotation.ID(int(i_angle + 540 - t_angle) % 360 - 180));
	l_motor.Set_speed(spd - motor_rotation.ID(int(i_angle + 540 - t_angle) % 360 - 180));
	r_motor.start_rotation();
	l_motor.start_rotation();
	if(abs(t_dist) <= 2 && abs(spd) < max_spd){
		complete = 1;
	}
}
char Task::Condition(){
	return condition;
}
void Task::utd(int a){
	t_dist = a;
}
void Task::uzd(int a){
	z_dst = a;
}
double* Task::pid(){
 	arr[0] = motor_rotation.P();
 	arr[1] = motor_rotation.I();
 	arr[2] = motor_rotation.D();
 	return arr;
}
void Task::mta(){
	int spd = int(speed_rotation.ID(t_dist));
	condition = 'm';
	r_motor.Set_speed(spd + motor_rotation.ID(int(i_angle + 540 - t_angle) % 360 - 180));
	l_motor.Set_speed(spd - motor_rotation.ID(int(i_angle + 540 - t_angle) % 360 - 180));
	r_motor.start_rotation();
	l_motor.start_rotation();
}
Task::~Task(){
	delete [] arr; 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
