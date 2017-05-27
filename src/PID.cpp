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
  
  // Set train_flag to true to increase changes in PID coefficients/weights
  bool train_flag = false;
  
  if (train_flag == true) {
    // If training, only use first 1000 iterations
    if (iter < 1000) {
      // Higher learning rate when training
      TweakK(0.001, cte);
    }
  } else {
    // Lower learning rate for test run, just makes very small tweaks
    TweakK(0.0001, cte);
  }
  
  // Raise number of iterations so that training can cut off
  iter += 1;
  
}

void PID::TweakK(float learn_rate, double cte) {
  /*
  * These tweak the PID coefficients/weights based on the CTE.
  * Note: If the training flag is set to "True" in UpdateWeights,
  * please comment out two of the coefficients to tweak only one
  * at once. Doing all three simultaneously can cause very eratic
  * driving behavior.
  */
  
  // Ki gets a special equation because it goes haywire with the other
  Kp -= learn_rate * cte;
  Ki += (learn_rate * cte) / i_error;
  Kd -= learn_rate * cte;
  
}

double PID::TotalError() {
  
  // Return the total error of each coefficient multiplied by the respective error
  return -Kp * p_error - Kd * d_error - Ki * i_error;
  
}

