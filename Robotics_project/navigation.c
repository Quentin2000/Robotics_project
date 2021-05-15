/*
 * navigation.c
 *
 *  Created on: 22 Apr 2021
 *      Authors: Jonathan Wei & Quentin Delfosse
 */

#include "hal.h"
#include <sensors/imu.h>
#include <motors.h>
#include <math.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <navigation.h>
#include <led_control.h>

#define BASIC_SPEED 200
#define MAX_SPEED 1000
#define DIST_THRESHOLD_MM 80 //threshold value for time of flight in mm
#define FORWARD_ANGLE_THRESHOLD 5*M_PI/6
#define TURNING_ANGLE_THRESHOLD 4*M_PI/6
#define THRESHOLD_STOPPED 1.5 //threshold to start moving while stopped
#define THRESHOLD_MOVING 0.6 // threshold to stop while moving
static thread_t *navThd;

enum {MOVING = 0, BREAKING = 1, STOPPED = 2, LEFT_TURN = 3, RIGHT_TURN = 4};

void direction(imu_msg_t *imu_values);


/**
 * @brief   Thread which controls the motors and lights depending on IMU inputs
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

	static float acc_threshold = THRESHOLD_STOPPED;

	//boolean indicating that the robot just started moving forward (used to reduce vibrations at the start)
	static uint8_t first_forward = 1;

	//creates a pointer to the array for shorter name
	float *accel = imu_values->acceleration;

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

		acc_threshold = THRESHOLD_MOVING;

		static float angle_threshold = FORWARD_ANGLE_THRESHOLD;

		//clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
		float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);

		//if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
		//so we correct it
		if(angle > M_PI){
			angle = -2 * M_PI + angle;
		}

		if(angle>angle_threshold || angle<-angle_threshold) {
			angle_threshold = TURNING_ANGLE_THRESHOLD;

			if (VL53L0X_get_dist_mm() > DIST_THRESHOLD_MM) { //move forward
				left_motor_set_speed(max_value(200*fabs(accel[Y_AXIS]),MAX_SPEED));
				right_motor_set_speed(max_value(200*fabs(accel[Y_AXIS]),MAX_SPEED));
				led_control(MOVING);

				if (first_forward){
		    		chThdSleepMilliseconds(300); //avoids stopping in case of sudden swinging from front to rear at the start
		    		first_forward = 0;
		    	}
			}
			else { //stop
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				led_control(STOPPED);
				first_forward = 1;
			}
		}
		else if(angle>=0) { //turn left
			angle_threshold = FORWARD_ANGLE_THRESHOLD;
			left_motor_set_speed(-BASIC_SPEED-(M_PI-angle)*500/M_PI);
			right_motor_set_speed(BASIC_SPEED+(M_PI-angle)*500/M_PI);
			led_control(LEFT_TURN);
			first_forward = 1;
		}
		else if(angle<0) { //turn right
			angle_threshold = FORWARD_ANGLE_THRESHOLD;
			left_motor_set_speed(BASIC_SPEED-(-M_PI-angle)*500/M_PI);
			right_motor_set_speed(-BASIC_SPEED+(-M_PI-angle)*500/M_PI);
			led_control(RIGHT_TURN);
			first_forward = 1;
		}
	}
	else { //stop
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		led_control(STOPPED);
		acc_threshold = THRESHOLD_STOPPED;
		first_forward = 1;
	}
}

float max_value(float a, float b) {
	return a < b ? a:b;
}
