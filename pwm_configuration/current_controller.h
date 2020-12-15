/*
 * current_controller.h
 *
 *  Created on: 15.12.2020
 *      Author: dvarx
 */

#ifndef CURRENT_CONTROLLER_H_
#define CURRENT_CONTROLLER_H_

//discrete controller state variables
struct pi_controller_32{
    float state_err_int;
    float e;
    float u;
    float kp;
    float ki;
    float delta_T;
};

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


#endif /* CURRENT_CONTROLLER_H_ */
