#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// Assign to instance variables
	this -> Kp = Kp;
	this -> Ki = Ki;
	this -> Kd = Kd;

	// Initialize errors to zero
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	// Twiddle vars initialization
	tol = 0.005;

	iter = 0;

    dp[0] = Kp * 0.1;	
  	dp[1] = Ki * 0.1;	
  	dp[2] = Kd * 0.1;


}

void PID::UpdateError(double cte) {
	// Differential Error
	d_error = cte - p_error;
	// Proportional Error
	p_error = cte;
	// Integral Error
	i_error += cte;
	iter += 1;

}

double PID::TotalError() {

	return CalculateTotalError(Kp, Ki, Kd);
}

double PID::CalculateTotalError(double k_p, double k_i, double k_d){
	double steer = - k_p * p_error - k_i * i_error - k_d * d_error;

	// Keep within the [-1, 1] steer value of the simulator
	if (steer < -1 ){
		steer = -1;
	} else if( steer > 1 ) {
		steer = 1;
	}

	return steer;
}

void PID::Twiddle(float tol, double steering_angle){
	double best_err = abs(TotalError());

	double best_cte = p_error;

	double rho = p_error / sin(steering_angle);
	double err;
	double err_d;
	double new_cte;

	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;

	// Loop
	while ((abs(dp[0]) + abs(dp[1]) + abs(dp[2])) > tol){
		// For each p
		for(int i = 0; i < 3; ++i){
			p[i] += dp[i];
			// Error
			err = abs(CalculateTotalError(p[0], p[1], p[2]));
			// Convert to Degrees
			err_d = err * (180 / M_PI);
			// Assign CTE
			new_cte = best_cte + (rho * sin(err_d));
			if((abs(new_cte) < abs(best_cte)) && (abs(err) < abs(best_err))){
				best_err = err;
				best_cte = new_cte;
				dp[i] *= 1.1;
			} else{
				// Search in the negative (other) direction
				p[i] -= 2 * dp[i];
				err = abs(CalculateTotalError(p[0], p[1], p[2]));
				err_d = err * (180 / M_PI);	
        		new_cte = best_cte + (rho * sin(err_d));	
        		if ((abs(new_cte) < abs(best_cte)) && (abs(err) < abs(best_err))){
        			best_cte = new_cte;	
		         	best_err = err;	
		         	dp[i] *= 1.1;	
        		} else {
        			p[i] += dp[i];
                 	dp[i] *= 0.9;
        		}
			}
		}
	}
   // Dealing with the - coeff
  for (int z = 0; z < 3; ++z) {	
    if (p[z] < 0.0) {	
      p[z] = abs(p[z]);	
    }	
  }	
  	
  // Reinitialize
  Kp = p[0];	
  Ki = p[1];	
  Kd = p[2];	
  	
  dp[0] = Kp * 0.1;	
  dp[1] = Ki * 0.1;	
  dp[2] = Kd * 0.1;

}



// void PID::Twiddler(float tol, double delta_p, double params){
// 	// # Don't forget to call `make_robot` before every call of `run`!
//     // p = [0, 0, 0]
//     // dp = [1, 1, 1]
//     double p = params;

//     robot = make_robot()
//     x_trajectory, y_trajectory, best_err = run(robot, p)
//     # TODO: twiddle loop here
//     iter = 0
//     while sum(p) > tol:
//         for i in range(len(p)):
//             p[i] += dp[i]
//             robot = make_robot()
//             x_trajectory, y_trajectory, err = run(robot, p)
//             if err < best_err:
//                 best_err = err
//                 dp[i] *= 1.1
//             else:
//                 p[i] -= 2 * dp[i]
//                 robot = make_robot()
//                 x_trajectory, y_trajectory, err = run(robot, p)
                
//                 if err < best_err:
//                     best_err = err
//                     dp[i] *= 1.1
//                 else:
//                     p[i] += dp[i]
//                     dp[i] *= 0.9
//         iter += 1
//     return p, best_err
// }