#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <navigation.h>
#include <sensors/imu.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>

#define NB_SAMPLES_OFFSET     200

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}



int main(void)
{

	halInit();

	 // System init
	chSysInit();
	mpu_init();
	// Inits the Inter Process Communication bus.
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	serial_start();
    usb_start();

    // starts the IMU
    imu_start();
    //starts the time of flight sensor
    VL53L0X_start();
    //initializes the motors
    motors_init();
    //starts the navigation thread
    navigation_start();

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	//wait 2 sec to be sure the e-puck is in a stable position
	chThdSleepMilliseconds(2000);

	while(1){
		//wait for new measures to be published
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		//prints values in readable units
		chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
				imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
				imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
				imu_values.status);

		chThdSleepMilliseconds(100);
	}

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
