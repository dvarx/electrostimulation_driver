/*
 * current_controller.h
 *
 *  Created on: 15.12.2020
 *      Author: dvarx
 */

#ifndef CURRENT_CONTROLLER_H_
#define CURRENT_CONTROLLER_H_

//discrete controller state variables
//u = kp*e + ki*integral(e)dt
struct pi_controller_32{
    float state_err_int;
    float e;
    float u;
    float kp;
    float ki;
    float delta_T;
};

inline void update_pi(struct pi_controller_32* ctrl,float err){
    //integral(e)[n]+=dt*e[n-1]
    ctrl->state_err_int+=ctrl->delta_T*ctrl->e;
    //ctrl->e := e[n]
    ctrl->e=err;
    //ctrl->u := u[n]
    ctrl->u=ctrl->kp*ctrl->e+ctrl->ki*ctrl->state_err_int;
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


#endif /* CURRENT_CONTROLLER_H_ */
