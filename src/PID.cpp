#include <vector>
#include <iostream>
#include <cmath>
#include "PID.h"

/*
* Initialize PID control, update the cross-track error, update PID
* coefficients, and calculate the total error (steering angle).
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /*
  * Initialize PID controller with the coefficients as the input values.
  * Note that the inputs to the function need different names than the
  * class variables or this will not work correctly.
  */
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // Initialize iterations for tweaking coefficients below.
  iter = 0;
  
  // Re-size Twiddle vectors
  p.resize(3);
  dp.resize(3);
  
  // Initialize Twiddle dp values
  dp[0] = 1.0;
  dp[1] = 1.0;
  dp[2] = 1.0;
  
}

void PID::UpdateError(double cte) {
  /*
  * Updates error values for calculating total error below. Also, updates
  * the error coefficients/weights based on whether full training is being
  * done or a test run.
  * Please note that it is advised to only do the training
  * on a single coefficient at a time, as the car will likely
  * otherwise run off the track.
  */
  
  // d_error is difference from old cte (p_error) to the new cte
  d_error = cte - p_error;
  // p_error gets set to the new cte
  p_error = cte;
  // i_error is the sum of ctes to this point
  i_error += cte;
  
  // Raise number of iterations so that Twiddle can cut off
  iter += 1;
  
}

void PID::Twiddle(double tolerance) {
  /*
  * These tweak the PID coefficients/weights based on the CTE.
  * Note: If the training flag is set to "True" in UpdateWeights,
  * please comment out two of the coefficients to tweak only one
  * at once. Doing all three simultaneously can cause very eratic
  * driving behavior.
  */
  
  double angle = std::abs(TotalError(Kp, Ki, Kd));
  // Kp is cte, which is currently the best error
  double best_err = Kp;
  double err;
  
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  
  // Twiddle loop
  while ((dp[0]+dp[1]+dp[2]) > tolerance) {
    for (int i = 0; i < p.size(); ++i) {
      p[i] += dp[i];
      err = std::abs(TotalError(p[0], p[1], p[2]));
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];
        err = std::abs(TotalError(p[0], p[1], p[2]));
        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.05;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.95;
        } // end inner if/else
      } // end outer if/else
    } // end for loop
  } // end while loop
              
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  
  //dp[0] = 1.0;
  //dp[1] = 1.0;
  //dp[2] = 1.0;

}

double PID::TotalError(double p_coeff, double i_coeff, double d_coeff) {
  
  // Return the total error of each coefficient multiplied by the respective error
  return -p_coeff * p_error - d_coeff * d_error - i_coeff * i_error;
  
}

