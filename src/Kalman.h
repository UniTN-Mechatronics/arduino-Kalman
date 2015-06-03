#ifndef __KALMAN_LIB__
#define __KALMAN_LIB__

class Kalman{

  public:
    Kalman();
    Kalman(float, float, float[2][2]);
    float update(float, float, float);
  private:
    // PARAMETERS
    float _Q_theta;        // kalmann filter angle covariance
    float _Q_thetad_bias;  // kalmann filter angular velocity convariance
    float _R;              // kalmann filter measure variance
    // ESTIMATED VARIABLES
    float _theta_est;
    float _thetad_bias_est;
    // INTERNAL FILTER VARIABLES
    float _P[2][2];
    float _y;
    float _S;
    float _K[2];
};

#endif
