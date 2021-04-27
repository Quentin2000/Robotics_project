/*
 * imu_control.c
 *
 *  Created on: 22 Apr 2021
 *      Author: jonathanwei
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <sensors/imu.h>
#include <motors.h>
#include <math.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define BASIC_SPEED 200
#define MAX_SPEED 1000
#define DIST_THRESHOLD_MM 50 //threshold value for time of flight in mm
static thread_t *navThd;

void direction(imu_msg_t *imu_values);

/**
 * @brief   Thread which updates the measures and publishes them
 */
static THD_WORKING_AREA(navigation_thd_wa, 512);
static THD_FUNCTION(navigation_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     imu_msg_t imu_values;

     while (1) {
    	 //if (VL53L0X_get_dist_mm() > DIST_THRESHOLD_MM) {
    	 //chprintf((BaseSequentialStream *)&SD3, "true");
    		 imu_values.acceleration[X_AXIS] = get_acceleration(X_AXIS);
    		 imu_values.acceleration[Y_AXIS] = get_acceleration(Y_AXIS);
    		 imu_values.acceleration[Z_AXIS] = get_acceleration(Z_AXIS);
    		 direction(&imu_values);
    	 chThdSleepMilliseconds(100);
    	 //}
     }
}

void navigation_start(void){

	navThd = chThdCreateStatic(navigation_thd_wa,
                     sizeof(navigation_thd_wa),
                     NORMALPRIO+1,
					 navigation_thd,
                     NULL);
}


void direction(imu_msg_t *imu_values) {
	//threshold value to not use the leds when the robot is too horizontal
	float threshold = 1;
	//create a pointer to the array for shorter name
	float *accel = imu_values->acceleration;

	/*
	*   example 1 with trigonometry.
	*/

	/*
	* Quadrant:
	*
	*       BACK
	*       ####
	*    #    0   #
	*  #            #
	* #-PI/2 TOP PI/2#
	* #      VIEW    #
	*  #            #
	*    # -PI|PI #
	*       ####
	*       FRONT
	*/

	if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold){

		//clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
		float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);

		//rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
		//angle += M_PI/4;
		//angle -= M_PI;

		//if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
		//so we correct it
		if(angle > M_PI){
			angle = -2 * M_PI + angle;
		}

		//chprintf((BaseSequentialStream *)&SD3, "%Angle=%-7d", angle);
		/*if(angle >= 0 && angle < M_PI/2){
			chprintf((BaseSequentialStream *)&SD3, "backward");
			backward = 1;
		}else if(angle >= M_PI/2 && angle < M_PI){
			chprintf((BaseSequentialStream *)&SD3, "left");
			left = 1;
		}else if(angle >= -M_PI && angle < -M_PI/2){
			chprintf((BaseSequentialStream *)&SD3, "forward");
			forward = 1;
		}else if(angle >= -M_PI/2 && angle < 0){
			chprintf((BaseSequentialStream *)&SD3, "right");
			right = 1;*/

		if(angle>11*M_PI/12 || angle<-11*M_PI/12) {
			//if (VL53L0X_get_dist_mm() > DIST_THRESHOLD_MM) {
				left_motor_set_speed(MAX_SPEED);
				right_motor_set_speed(MAX_SPEED);
			//}
		}
		else if(angle>=0) {
			left_motor_set_speed(-BASIC_SPEED-(M_PI-angle)*500/M_PI);
			right_motor_set_speed(BASIC_SPEED+(M_PI-angle)*500/M_PI);
		}
		else if(angle<0) {
			left_motor_set_speed(BASIC_SPEED-(-M_PI-angle)*500/M_PI);
			right_motor_set_speed(-BASIC_SPEED+(-M_PI-angle)*500/M_PI);
		}
	}
	else /*if(fabs(accel[X_AXIS]) < threshold && fabs(accel[Y_AXIS]) < threshold)*/ {
		chprintf((BaseSequentialStream *)&SD3, "Hello");
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
}
