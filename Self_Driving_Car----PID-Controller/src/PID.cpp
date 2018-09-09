#include <iostream>
#include <limits>
#include "PID.h"

using namespace std;

// Program by 	NIKHIL XAVIER
// Program written on 14th February 2018

PID::PID() 
{

}

PID::~PID() 
{

}

void PID::Init(double Kp_init, double Ki_init, double Kd_init) 
{
	this->Kp = Kp_init; 
	this->Ki = Ki_init; 
	this->Kd = Kd_init;
	double max_value;
	max_value = numeric_limits<double>::max(); 
	p_error = max_value;  
	i_error = 0;
	d_error = 0; 
}

void PID::UpdateError(double cte_temp) 
{
	double temp;
	temp = numeric_limits<double>::max();
	if (p_error == temp)
	{
		p_error = cte_temp;
	}
	d_error = cte_temp - p_error; 
	p_error = cte_temp;
	i_error = i_error + cte_temp; 
}

double PID::TotalError() 
{
	double total_error_temp;
	total_error_temp = Kp * p_error + Kd * d_error + Ki * i_error;	
	return (total_error_temp);
}

