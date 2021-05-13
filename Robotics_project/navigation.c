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
#include <led_control.h>

#define BASIC_SPEED 200
#define MAX_SPEED 1000
#define DIST_THRESHOLD_MM 80 //threshold value for time of flight in mm
static thread_t *navThd;

void direction(imu_msg_t *imu_values);
float max_value(float a, float b);

enum {MOVING = 0, BREAKING = 1, STOPPED = 2, LEFT_TURN = 3, RIGHT_TURN = 4};

/**
 * @brief   Thread which updates the measures and publishes them
 */
static THD_WORKING_AREA(navigation_thd_wa, 512);
static THD_FUNCTION(navigation_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     imu_msg_t imu_values;

     while (1) {

		 imu_values.acceleration[X_AXIS] = get_acceleration(X_AXIS);
		 imu_values.acceleration[Y_AXIS] = get_acceleration(Y_AXIS);

		 direction(&imu_values);

    	 chThdSleepMilliseconds(100);

     }
}

void navigation_start(void){

	navThd = chThdCreateStatic(navigation_thd_wa,
                     sizeof(navigation_thd_wa),
                     NORMALPRIO +11,
					 navigation_thd,
                     NULL);
}


void direction(imu_msg_t *imu_values) {

	//threshold value to not move when the robot is too horizontal
	static float acc_threshold = 1;
	static uint8_t first_forward = 1;
	chprintf((BaseSequentialStream *)&SD3, "%acc_threshold=%-7f", acc_threshold);
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

	if(fabs(accel[X_AXIS]) > acc_threshold || fabs(accel[Y_AXIS]) > acc_threshold){

		acc_threshold = 0.6; // 0.6 because of the difference between front support and rear support
		static float angle_threshold = 5*M_PI/6;

		//clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
		float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);

		//if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
		//so we correct it
		if(angle > M_PI){
			angle = -2 * M_PI + angle;
		}

		if(angle>angle_threshold || angle<-angle_threshold) {
			angle_threshold = 4*M_PI/6;
			if (VL53L0X_get_dist_mm() > DIST_THRESHOLD_MM) {
				left_motor_set_speed(max_value(200*fabs(accel[Y_AXIS]),MAX_SPEED));
				right_motor_set_speed(max_value(200*fabs(accel[Y_AXIS]),MAX_SPEED));
				led_control(MOVING);
				if (first_forward){
		    		chThdSleepMilliseconds(300);
		    		first_forward = 0;
		    	}
			}
			else {
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				led_control(STOPPED);
				first_forward = 1;
			}
		}
		else if(angle>=0) {
			angle_threshold = 5*M_PI/6;
			left_motor_set_speed(-BASIC_SPEED-(M_PI-angle)*500/M_PI);
			right_motor_set_speed(BASIC_SPEED+(M_PI-angle)*500/M_PI);
			led_control(LEFT_TURN);
			first_forward = 1;
		}
		else if(angle<0) {
			angle_threshold = 5*M_PI/6;
			left_motor_set_speed(BASIC_SPEED-(-M_PI-angle)*500/M_PI);
			right_motor_set_speed(-BASIC_SPEED+(-M_PI-angle)*500/M_PI);
			led_control(RIGHT_TURN);
			first_forward = 1;
		}
	}
	else /*if(fabs(accel[X_AXIS]) < acc_threshold && fabs(accel[Y_AXIS]) < acc_threshold)*/ {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		led_control(STOPPED);
		acc_threshold = 1.5;
		first_forward = 1;
	}
}


float max_value(float a, float b) {
	return a < b ? a:b;
}
