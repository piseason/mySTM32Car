#ifndef __M5StickC_H
#define __M5StickC_H

extern float thetaP, thetaI, thetaD, power;
extern float theta0, theta_dot0, theta, theta_dot, Kp, Ki, Kd;
extern float Kvp, Kvi, Kvd, thetaII, thetaOffset,thetaV;

void ini_theta(void);
void update_theta(void);
void update_theta2(float thetaTar);
void angleFilter(float tmpI);
void angleCorrection(void);
void update_thetaEx(float thetaTar);
#endif
