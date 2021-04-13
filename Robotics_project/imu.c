#include <ch.h>
#include <hal.h>
#include <math.h>
#include <msgbus/messagebus.h>
#include <imu.h>

#define STANDARD_GRAVITY    9.80665f
#define DEG2RAD(deg) (deg / 180 * M_PI)

#define RES_2G		2.0f
#define RES_250DPS	250.0f
#define MAX_INT16	32768.0f

#define ACC_RAW2G	 (RES_2G / MAX_INT16)	//2G scale for 32768 raw value
#define GYRO_RAW2DPS (RES_250DPS / MAX_INT16)	//250DPS (degrees per second) scale for 32768 raw value

extern messagebus_t bus;

static imu_msg_t imu_values;

static thread_t *imuThd;
static bool imu_configured = false;

/***************************INTERNAL FUNCTIONS************************************/

 /**
 * @brief   Computes the measurements of the imu into readable measurements
 * 			RAW accelerometer to m/s^2 acceleration
 * 			RAW gyroscope to rad/s speed
 */
void imu_compute_units(void){
	for(uint8_t i = 0 ; i < NB_AXIS ; i++){
		imu_values.acceleration[i] = ( (imu_values.acc_raw[i] - imu_values.acc_offset[i]) 
							         * STANDARD_GRAVITY * ACC_RAW2G);
		imu_values.gyro_rate[i] = ( (imu_values.gyro_raw[i] - imu_values.gyro_offset[i]) 
								  * DEG2RAD(GYRO_RAW2DPS) );
	}
}//test

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the motors in order to turn them off at each loop and to
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //threshold value to not use the motors when the robot is too horizontal
    float threshold = 0.2;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;


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

        chSysLock();
        //reset the timer counter
        GPTD11.tim->CNT = 0;
        //clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
        float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);
        //by reading time with the debugger, we can know the computational time of atan2 function
        time = GPTD11.tim->CNT;
        chSysUnlock();

        //rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
        angle += M_PI/4;

        //if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
        //so we correct it
        if(angle > M_PI){
            angle = -2 * M_PI + angle;
        }

        if(angle >= 0 && angle < M_PI/2){
            led5 = 1;
        }else if(angle >= M_PI/2 && angle < M_PI){
            led7 = 1;
        }else if(angle >= -M_PI && angle < -M_PI/2){
            led1 = 1;
        }else if(angle >= -M_PI/2 && angle < 0){
            led3 = 1;
        }
    }

    /*
     *   example 2 with only conditions
     */

    // chSysLock();
    // GPTD11.tim->CNT = 0;

    // //we find which led of each axis should be turned on
    // if(accel[X_AXIS] > threshold)
    //     led7 = 1;
    // else if(accel[X_AXIS] < -threshold)
    //     led3 = 1;

    // if(accel[Y_AXIS] > threshold)
    //     led5 = 1;
    // else if(accel[Y_AXIS] < -threshold)
    //     led1 = 1;

    // //if two leds are turned on, turn off the one with the smaller
    // //accelerometer value
    // if(led1 && led3){
    //     if(accel[Y_AXIS] < accel[X_AXIS])
    //         led3 = 0;
    //     else
    //         led1 = 0;
    // }else if(led3 && led5){
    //     if(accel[X_AXIS] < -accel[Y_AXIS])
    //         led5 = 0;
    //     else
    //         led3 = 0;
    // }else if(led5 && led7){
    //     if(accel[Y_AXIS] > accel[X_AXIS])
    //         led7 = 0;
    //     else
    //         led5 = 0;
    // }else if(led7 && led1){
    //     if(accel[X_AXIS] > -accel[Y_AXIS])
    //         led1 = 0;
    //     else
    //         led7 = 0;
    // }
    // time = GPTD11.tim->CNT;
    // chSysUnlock();

    //to see the duration on the console
    chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}

 /**
 * @brief   Thread which updates the measurements and publishes them
 */
static THD_FUNCTION(imu_reader_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     // Declares the topic on the bus.
     messagebus_topic_t imu_topic;
     MUTEX_DECL(imu_topic_lock);
     CONDVAR_DECL(imu_topic_condvar);
     messagebus_topic_init(&imu_topic, &imu_topic_lock, &imu_topic_condvar, &imu_values, sizeof(imu_values));
     messagebus_advertise_topic(&bus, &imu_topic, "/imu");

     systime_t time;

     while (chThdShouldTerminateX() == false) {
    	 time = chVTGetSystemTime();

    	if(imu_configured == true){
	 		/* Reads the incoming measurement. */
			mpu9250_read(imu_values.gyro_raw, imu_values.acc_raw, &imu_values.status);
			/* computes the raw values into readable values*/
			imu_compute_units();
     	}

     	/* Publishes it on the bus. */
		messagebus_topic_publish(&imu_topic, &imu_values, sizeof(imu_values));

        chThdSleepUntilWindowed(time, time + MS2ST(4)); //reduced the sample rate to 250Hz

     }
}

/*************************END INTERNAL FUNCTIONS**********************************/


/****************************PUBLIC FUNCTIONS*************************************/

void imu_start(void)
{
	int8_t status = MSG_OK;

    status = mpu9250_setup(MPU9250_ACC_FULL_RANGE_2G
		                  | MPU9250_GYRO_FULL_RANGE_250DPS
		                  | MPU9250_SAMPLE_RATE_DIV(100));
		                  //| MPU60X0_LOW_PASS_FILTER_6)

    //not tested yet because the auxilliary I2C of the MPU-9250 is condamned due
    //to PCB correction on the e-puck2-F4, so the magnetometer cannot be read...
    // if(status == MSG_OK){
    // 	status = mpu9250_magnetometer_setup();
    // }

    if(status == MSG_OK){
    	imu_configured = true;
    }

    static THD_WORKING_AREA(imu_reader_thd_wa, 1024);
    imuThd = chThdCreateStatic(imu_reader_thd_wa, sizeof(imu_reader_thd_wa), NORMALPRIO, imu_reader_thd, NULL);
}

void imu_stop(void) {
    chThdTerminate(imuThd);
    chThdWait(imuThd);
    imuThd = NULL;
}

void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples){

    //creates temporary array used to store the sum for the average
    int32_t temp_acc_offset[NB_AXIS] = {0};
    int32_t temp_gyro_offset[NB_AXIS] = {0};

    //sums nb_samples
    for(uint16_t i = 0 ; i < nb_samples ; i++){
        //waits for new measurements for IMU using MessageBus library
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
        for(uint8_t j = 0 ; j < NB_AXIS ; j++){
            temp_acc_offset[j] += imu_values.acc_raw[j];
            temp_gyro_offset[j] += imu_values.gyro_raw[j];
        }
    }
    //finishes the average by dividing the sums by nb_samples
    //then stores the values to the good fields of imu_values to keep them
    for(uint8_t j = 0 ; j < NB_AXIS ; j++){
        temp_acc_offset[j]  /= nb_samples;
        temp_gyro_offset[j] /= nb_samples;

        imu_values.acc_offset[j] = temp_acc_offset[j];
        imu_values.gyro_offset[j] = temp_gyro_offset[j];
    }
    //specific case for the z axis because it should not be zero but -1g
    //deletes the standard gravity to have only the offset
    imu_values.acc_offset[Z_AXIS] += (MAX_INT16 / RES_2G); //16384 = 1g with a scale of 2G
}

int16_t get_acc(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.acc_raw[axis];
	}
	return 0;
}

void get_acc_all(int16_t *values) {
	values[X_AXIS] = imu_values.acc_raw[X_AXIS];
	values[Y_AXIS] = imu_values.acc_raw[Y_AXIS];
	values[Z_AXIS] = imu_values.acc_raw[Z_AXIS];
}


int16_t get_acc_offset(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.acc_offset[axis];
	}
	return 0;
}

float get_acceleration(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.acceleration[axis];
	}
	return 0;
}


int16_t get_gyro(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.gyro_raw[axis];
	}
	return 0;
}

void get_gyro_all(int16_t *values) {
	values[X_AXIS] = imu_values.gyro_raw[X_AXIS];
	values[Y_AXIS] = imu_values.gyro_raw[Y_AXIS];
	values[Z_AXIS] = imu_values.gyro_raw[Z_AXIS];
}

int16_t get_gyro_offset(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.gyro_offset[axis];
	}
	return 0;
}

float get_gyro_rate(uint8_t axis) {
	if(axis < NB_AXIS) {
		return imu_values.gyro_rate[axis];
	}
	return 0;
}

float get_temperature(void) {
	return imu_values.temperature;
}

/**************************END PUBLIC FUNCTIONS***********************************/

