#ifndef PID_H
#define PID_H
#include <cmath>

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
    Errors
  */


  /*
  Twiddle 
  */
  // Tolerance
  float tol;
  double dp[3];
  // 
  double p[3];
  // iterations
  int iter;
  double cte;
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
  * Calculate the total PID error.
  */
  double TotalError();

  double CalculateTotalError(double k_p, double k_i, double k_d);
  /*
  Twiddler method for tuning the params
  */
  void Twiddle(float tol, double steering_angle);
};

#endif /* PID_H */
