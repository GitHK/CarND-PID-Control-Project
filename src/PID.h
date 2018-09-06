#ifndef PID_H
#define PID_H

enum coefficient_type {
    P,
    I,
    D
};


class PID {
public:

    // twiddle
    bool useTwiddle;
    double tolerance;
    double prev_cte;

    static constexpr int N = 3;
    int current_param;  // the current parameter which is being tweedled
    double best_error;
    double total_error;
    bool run_robot_again;

    double p[N];            // coefficients
    double dp[N];           //
    double errors[N];   // errors

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
    void Init(double Kp, double Ki, double Kd, bool useTwiddle);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    /*
     * Use this to optimise coefficients of the PID
     * */
    void TwiddleCoefficients();

    double UsableOutput();
};

#endif /* PID_H */
