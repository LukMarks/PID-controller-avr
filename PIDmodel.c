#include <avr/io.h>

double sensed_output, control_signal;
double setpoint;
double Kp; //proportional gain
double Ki; //integral gain
double Kd; //derivative gain
int T; //sample time in milliseconds (ms)
unsigned long last_time_sample;
double total_error, last_error;
int max_control; // boundary of maximum control input
int min_control; // boundary of minimum control input

void setup(){
	
}

void loop(){
	
	PID_Control(); 
	
}

void PID_Control(){

	unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
	
	int delta_time = current_time - last_time_sample; //delta time interval
	
	if (delta_time >= T){

		double error = setpoint - sensed_output;
		
		total_error += error; //accumulates the error - integral operation term
		
		// avoid integral windup effect
		if (total_error >= max_control) total_error = max_control;  
		else if (total_error <= min_control) total_error = min_control;
		
		double delta_error = error - last_error; //difference of error for derivative term

		control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control calculation
		
		// avoid integral windup effect
		if (control_signal >= max_control) control_signal = max_control;
		else if (control_signal <= min_control) control_signal = min_control;
		
		// update error's variables for the next iteration.
		last_error = error;
		last_time_sample = current_time;
	}
}