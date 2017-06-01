#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
  * Iterations
  */
  int iter;
  
  /*
  * Twiddle values
  */
  std::vector <double> p;
  std::vector <double> dp;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  /*
  * Tweaks the PID coefficients/weights for better driving.
  */
  void Twiddle(double tolerance);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double p_coeff, double i_coeff, double d_coeff);
};

#endif /* PID_H */
