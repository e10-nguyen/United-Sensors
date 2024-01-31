/*
 * barometer.h
 *
 *  Created on: Dec 7, 2023
 *      Author: ethan
 */

#ifndef SRC_BAROMETER_H_
#define SRC_BAROMETER_H_

void DPS310_Start (void);

void DPS310_GetPress (I2C_HandleTypeDef* hi2c);

void DPS310_Poll (void);

#endif /* SRC_BAROMETER_H_ */
