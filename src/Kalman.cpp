#include "Kalman.h"

Kalman::Kalman(){
  _theta_est   = 0;
  _thetad_bias_est = 0;
  _P[0][0]  = 10;
  _P[1][0]  = 0;
  _P[0][1]  = 0;
  _P[1][1]  = 10;
}

Kalman::Kalman(float theta_est_0, float thetad_bias_est_0, float P_0[2][2]){
  _theta_est = theta_est_0;
  _thetad_bias_est = thetad_bias_est_0;
  _P[0][0]  = P_0[0][0];
  _P[1][0]  = P_0[1][0];
  _P[0][1]  = P_0[0][1];
  _P[1][1]  = P_0[1][1];
}

float Kalman::update(float dt, float theta, float thetad) {
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

