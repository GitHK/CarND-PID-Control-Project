#include <iostream>
#include "PID.h"


PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp, double Ki, double Kd, bool useTwiddle) {
    double percentage = 0.4;
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

    dp[0] = Kp * percentage;
    dp[1] = Ki * percentage;
    dp[2] = Kd * percentage;

    tolerance = 0.000001;
    current_param = 0;
    best_error = 10000;
    total_error = 0;
    run_robot_again = false;

    this->useTwiddle = useTwiddle;
}


void PID::UpdateError(double cte) {
    if (prev_cte == 0)
        prev_cte = cte;

    total_error += cte * cte;

    errors[0] = cte;  // p_error
    errors[1] += cte; // i_error
    errors[2] = cte - prev_cte;   // d_error

    prev_cte = cte;

    // run TWIDDLE on all params here with their respective DPs
    if (useTwiddle) {
        TwiddleCoefficients();
        std::cout << " I[" << current_param << "]"
                  << " Kp " << p[0] << " p_dp " << errors[0]
                  << " Ki " << p[1] << " i_dp " << errors[1]
                  << " Kd " << p[2] << " d_dp " << errors[2]
                  << std::endl;
    }
}

double PID::TotalError() {
    // used for the twiddle function
    return total_error;
}

double PID::UsableOutput() {
    // -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
    double output = -p[0] * errors[0] - p[1] * errors[1] - p[2] * errors[2];

    if (output > 1.) output = 1.;
    if (output < -1.) output = 1.;

    return output;
}

void PID::TwiddleCoefficients() {
    double dp_sum = dp[0] + dp[1] + dp[2];

    if (dp_sum > tolerance) {
        if (!run_robot_again) {
            p[current_param] += dp[current_param];
            // current robot run!
            double error = TotalError();
            if (error < best_error) {
                best_error = error;
                dp[current_param] *= 1.1;
            } else {
                p[current_param] -= 2 * dp[current_param];
                // run robot again and get error!
                run_robot_again = true;
            }
        } else {
            double error = TotalError();
            if (error < best_error) {
                best_error = error;
                dp[current_param] *= 1.1;
            } else {
                p[current_param] += dp[current_param];
                dp[current_param] *= 0.9;
            }

            // go to next param when finished
            current_param = (current_param + 1) % N;
            run_robot_again = false;
        }
    }
}
