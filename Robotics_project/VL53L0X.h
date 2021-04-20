/**
 * @file    VL53L0X.h
 * @brief   High level functions to use the VL53L0X TOF sensor.
 * 
 * @author  Eliot Ferragni
 */


#ifndef VL53L0X_H
#define VL53L0X_H

#include "Api/core/inc/vl53l0x_api.h"

#define USE_I2C_2V8

#define VL53L0X_ADDR 0x52

/** Message representing a measurement of the time of flight sensor. */
typedef struct {
    uint8_t move;
    uint16_t dist_mm;
} tof_msg_t;

//////////////////// PROTOTYPES PUBLIC FUNCTIONS /////////////////////

/**
 * @brief 			Init the VL53L0X_Dev_t structure and the sensor.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */	
VL53L0X_Error VL53L0X_init(VL53L0X_Dev_t* device);

/**
 * @brief 			Configure the accuracy of the sensor (range).
 * 
 * @param device 	Pointer to the structure of the sensor
 * @param accuracy 	Accuracy chosen
 * 
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_configAccuracy(VL53L0X_Dev_t* device, VL53L0X_AccuracyMode accuracy);

/**
 * @brief 			Begin the meausurement process with the specified mode.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @param mode 		Mode chosen to take the measures
 * 
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_startMeasure(VL53L0X_Dev_t* device, VL53L0X_DeviceModes mode);

/**
 * @brief 			Get the last valid measure and lpace it in the sensor structure given.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_getLastMeasure(VL53L0X_Dev_t* device);

/**
 * @brief 			Stop the measuement process.
 * 
 * @param device 	Pointer to the structure of the sensor
 * @return 			VL53L0X_Error See ::VL53L0X_Error
 */	
VL53L0X_Error VL53L0X_stopMeasure(VL53L0X_Dev_t* device);

/**
* @brief   Compares the distance measured with the defined threshold
* to compute the move variable (to be then shared to the motors).
*
*/
void compute_move(void);

/**
 * @brief Init a thread which uses the distance sensor to
 * continuoulsy measure the distance.
 */
void VL53L0X_start(void);

/**
* @brief   Stop the distance measurement.
*
*/
void VL53L0X_stop(void);

/**
 * @brief 			Return the last distance measured in mm
 * 
 * @return 			Last distance measured in mm
 */	
uint16_t VL53L0X_get_dist_mm(void);

/**
 * @brief 			Return the move order (used to control the motors)
 *
 * @return 			Move boolean
 */
//uint16_t VL53L0X_get_move(void);

#endif /* VL53L0X_H*/
