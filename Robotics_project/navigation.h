/*
 * imu_control.h
 *
 *  Created on: 22 Apr 2021
 *      Author: jonathanwei
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_


void navigation_start(void);
void motor_control(uint16_t *gyro_values_raw);


#endif /* NAVIGATION_H_ */
