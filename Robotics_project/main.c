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

/*static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,       // 1MHz timer clock in order to measure uS.
        NULL,           // Timer callback.
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}*/



int main(void)
{

	halInit();

	chSysInit();
	mpu_init();
	  /** Inits the Inter Process Communication bus. */
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	serial_start();
    usb_start();

    i2c_start();
    imu_start();

    //starts the time of flight sensor
    VL53L0X_start();
    //starts the navigation thread
    navigation_start();

    //starts the USB communication

    //starts timer 12
   // timer12_start();
    //inits the motors
    motors_init();


	 /* System init */







	    //to change the priority of the thread invoking the function. The main function in this case
	    //chThdSetPriority(NORMALPRIO+2);

	    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	    imu_msg_t imu_values;

	    //wait 2 sec to be sure the e-puck is in a stable position
	    chThdSleepMilliseconds(2000);
	   // imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);

	    while(1){
	        //wait for new measures to be published
	        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
	        //prints raw values
	        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
	                imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
	                imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

	        //prints raw values with offset correction
	        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
	                imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS],
	                imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS],
	                imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS],
	                imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS],
	                imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS],
	                imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS]);

	        //prints values in readable units
	        chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
	                imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
	                imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
	                imu_values.status);

	        //show_gravity(&imu_values);
	        chThdSleepMilliseconds(100);
	    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
