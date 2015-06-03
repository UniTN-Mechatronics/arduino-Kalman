#include "Kalman.h"

Kalman::Kalman(){
  double P[2][2];
  P[0][0] = 10;
  P[1][0] = 0;
  P[0][1] = 0;
  P[1][1] = 10;
  init(0.0, 0.0, P);
}

Kalman::Kalman(double theta_est_0) {
  double P[2][2];
  P[0][0] = 10;
  P[1][0] = 0;
  P[0][1] = 0;
  P[1][1] = 10;
  init(theta_est_0, 0.0, P);
}

Kalman::Kalman(double theta_est_0, double thetad_bias_est_0, double P_0[2][2]){
  init(theta_est_0, thetad_bias_est_0, P_0);
}

void Kalman::init(double theta_est_0, double thetad_bias_est_0, double P_0[2][2]) {
  _theta_est       = theta_est_0;
  _thetad_bias_est = thetad_bias_est_0;
  _P[0][0]       = P_0[0][0];
  _P[1][0]       = P_0[1][0];
  _P[0][1]       = P_0[0][1];
  _P[1][1]       = P_0[1][1];
  _Q_theta       = 0.0000001;
  _Q_thetad_bias = 0.00001;
  _R             = 2.0;
}


double Kalman::update(double dt, double theta, double thetad) {
  // estimate step
  _theta_est += dt*(thetad - _thetad_bias_est);
  _P[0][0] += dt*(_P[1][1]*dt - _P[0][1] - _P[1][0] + _Q_theta);
  _P[0][1] -= _P[1][1]*dt;
  _P[1][0] -= _P[1][1]*dt;
  _P[1][1] += _Q_thetad_bias*dt;
  // observation step
  _y = theta - _theta_est;
  _S = _P[0][0] + _R;
  // update step
  _K[0]       = _P[0][0]/_S;
  _K[1]       = _P[1][0]/_S;

  _theta_est    += _K[0]*_y;
  _thetad_bias_est += _K[1]*_y;

  _P[0][0]   -= _K[0]*_P[0][0];
  _P[0][1]   -= _K[0]*_P[0][1];
  _P[1][0]   -= _K[1]*_P[0][0];
  _P[1][1]   -= _K[1]*_P[0][1];

  return _theta_est;
}

