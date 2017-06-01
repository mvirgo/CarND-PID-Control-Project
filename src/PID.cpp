#include <vector>
#include <iostream>
#include <cmath>
#include "PID.h"

/*
* Initialize PID control, update the cross-track error, update PID
* coefficients, Twiddle, and calculate the total error (steering angle).
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
  
  // Initialize Twiddle dp values - normalized to be similar to coefficients
  dp[0] = Kp * 0.1;
  dp[1] = Ki * 0.1;
  dp[2] = Kd * 0.1;
  
}

void PID::UpdateError(double cte) {
  /*
  * Updates error values for calculating total error below.
  */
  
  // d_error is difference from old cte (p_error) to the new cte
  d_error = (cte - p_error);
  // p_error gets set to the new cte
  p_error = cte;
  // i_error is the sum of ctes to this point
  i_error += cte;
  
  // Raise number of iterations so that Twiddle can start & cut off approproately
  iter += 1;
  
}

void PID::Twiddle(double tolerance, double angle) {
  /*
  * These tweak the PID coefficients/weights based on the Total Error & a predicted CTE.
  */
  
  // Best error starts at current total error
  double best_err = std::abs(TotalError(Kp, Ki, Kd));
  // Best CTE starts at p_error, which is CTE (see UpdateError above)
  double best_cte = p_error;
  // Y = rho * sin(angle), whereby CTE is Y and we solve for rho for the cte prediction below
  double rho = p_error / sin(angle);
  // Other desired variables
  double err;
  double err_deg;
  double new_cte;
  
  // Set each p value to the PID coefficients
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  
  // Twiddle loop
  while ((std::abs(dp[0])+std::abs(dp[1])+std::abs(dp[2])) > tolerance) {
    // For each p value
    for (int i = 0; i < p.size(); ++i) {
      p[i] += dp[i];
      // Error is the turn rate
      err = std::abs(TotalError(p[0], p[1], p[2]));
      // Change to degrees
      err_deg = err * (180 / M_PI);
      // Predict CTE
      new_cte = best_cte + (rho * sin(err_deg));
      if ((std::abs(new_cte) < std::abs(best_cte)) && (std::abs(err) < std::abs(best_err))) {
        // If better than previous for both CTE and total error, change best values and expand the search further
        best_cte = new_cte;
        best_err = err;
        dp[i] *= 1.1;
      } else {
        // Flip the search in the opposite direction
        p[i] -= 2 * dp[i];
        err = std::abs(TotalError(p[0], p[1], p[2]));
        err_deg = err * (180 / M_PI);
        new_cte = best_cte+ (rho * sin(err_deg));
        if ((std::abs(new_cte) < std::abs(best_cte)) && (std::abs(err) < std::abs(best_err))) {
          best_cte = new_cte;
          best_err = err;
          dp[i] *= 1.1;
        } else {
          // Shrink the search if nothing better found
          p[i] += dp[i];
          dp[i] *= 0.9;
        }   // end inner if/else
      }   // end outer if/else
    }   // end for loop
  }   // end while loop
  
  // Negative coefficients would jack up the steering calculation, so force positive
  for (int z = 0; z < p.size(); ++z) {
    if (p[z] < 0.0) {
      p[z] = std::abs(p[z]);
    }
  }
  
  // Change class error coefficients to the p values from above after Twiddle loop
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  
  // Reset the dp values to be used again for another iteration
  dp[0] = Kp * 0.1;
  dp[1] = Ki * 0.1;
  dp[2] = Kd * 0.1;

}

double PID::TotalError(double p_coeff, double i_coeff, double d_coeff) {
  
  // Return the total error of each coefficient multiplied by the respective error
  return -p_coeff * p_error - d_coeff * d_error - i_coeff * i_error;
  
}

