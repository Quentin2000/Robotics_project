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
#include <sensors/VL53L0X/VL53L0X.h>

#define DIST_THRESHOLD_MM 20 //threshold value for time of flight in mm
static thread_t *navThd;
/**
 * @brief   Thread which updates the measures and publishes them
 */
static THD_WORKING_AREA(navigation_thd_wa, 512);
static THD_FUNCTION(navigation_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);


     while (1) {
    	 if (VL53L0X_get_dist_mm() > DIST_THRESHOLD_MM) {
    		 chprintf((BaseSequentialStream *)&SD3, "true");
    	 }
    	 chThdSleepMilliseconds(100);
     }
}

void navigation_start(void){

	navThd = chThdCreateStatic(navigation_thd_wa,
                     sizeof(navigation_thd_wa),
                     NORMALPRIO,
					 navigation_thd,
                     NULL);
}

