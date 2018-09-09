#ifndef PID_H
#define PID_H

// Program by 	NIKHIL XAVIER
// Program written on 14th February 2018

class PID 
{
public:
  double p_error;
  double i_error;
  double d_error;
  double Kp;
  double Ki;
  double Kd;
  PID();
  virtual ~PID();
  void Init(double Kp, double Ki, double Kd);
  void UpdateError(double cte);
  double TotalError();
};

#endif
