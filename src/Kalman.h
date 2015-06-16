#ifndef __KALMAN_LIB__
#define __KALMAN_LIB__

class Kalman {

public:
  Kalman();
  Kalman(double theta_est_0);
  Kalman(double theta_est_0, double thetad_bias_est_0, double P_0[2][2]);
  void init(double theta_est_0, double thetad_bias_est_0, double P_0[2][2]);
  double update(double, double, double);
  double theta_est() { return _theta_est; };
  double Q_theta() { return _Q_theta; };
  double Q_thetad_bias() { return _Q_thetad_bias; };
  void set_R(double val) { _R = val; };
  void set_Q_theta(double val) { _Q_theta = val; };
  void set_Q_thetad_bias(double val) { _Q_thetad_bias = val; };

private:
  // PARAMETERS
  double _Q_theta;       // kalmann filter angle covariance
  double _Q_thetad_bias; // kalmann filter angular velocity convariance
  double _R;             // kalmann filter measure variance
  // ESTIMATED VARIABLES
  double _theta_est;
  double _thetad_bias_est;
  // INTERNAL FILTER VARIABLES
  double _P[2][2];
  double _y;
  double _S;
  double _K[2];
};

#endif
